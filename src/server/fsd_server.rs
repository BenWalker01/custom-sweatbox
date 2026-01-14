use anyhow::{Result, Context};
use tokio::net::{TcpListener, TcpStream};
use tokio::io::AsyncReadExt;
use tokio::sync::Mutex;
use std::sync::Arc;
use tracing::{info, warn, error};

use super::controller_handler::ControllerHandler;
use super::pilot_handler::PilotHandler;
use super::message_handler::{MessageHandler, MessageStatus, ClientType};

/// Main FSD server
pub struct FsdServer {
    port: u16,
    host: String,
    controllers: Arc<Mutex<Vec<Arc<Mutex<ControllerHandler>>>>>,
    pilots: Arc<Mutex<Vec<Arc<Mutex<PilotHandler>>>>>,
}

impl FsdServer {
    /// Create a new FSD server
    pub fn new(host: String, port: u16) -> Self {
        Self {
            port,
            host,
            controllers: Arc::new(Mutex::new(Vec::new())),
            pilots: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Start the server
    pub async fn start(&self) -> Result<()> {
        let addr = format!("{}:{}", self.host, self.port);
        let listener = TcpListener::bind(&addr).await
            .context(format!("Failed to bind to {}", addr))?;

        info!("[LISTENING] Server is listening on {}", addr);

        loop {
            match listener.accept().await {
                Ok((stream, addr)) => {
                    info!("[NEW CONNECTION] {} connected", addr);
                    
                    let controllers = self.controllers.clone();
                    let pilots = self.pilots.clone();
                    
                    tokio::spawn(async move {
                        if let Err(e) = Self::handle_client(stream, addr.to_string(), controllers, pilots).await {
                            error!("[ERROR] Client handler error: {}", e);
                        }
                    });
                }
                Err(e) => {
                    error!("[ERROR] Failed to accept connection: {}", e);
                }
            }
        }
    }

    /// Handle a client connection
    async fn handle_client(
        stream: TcpStream,
        addr: String,
        controllers: Arc<Mutex<Vec<Arc<Mutex<ControllerHandler>>>>>,
        pilots: Arc<Mutex<Vec<Arc<Mutex<PilotHandler>>>>>,
    ) -> Result<()> {
        let mut buffer = vec![0u8; 262144];
        let mut first_message = true;
        let mut handler_type: Option<ClientType> = None;
        let mut controller_handler: Option<Arc<Mutex<ControllerHandler>>> = None;
        let mut pilot_handler: Option<Arc<Mutex<PilotHandler>>> = None;
        
        // We'll split the stream on first message
        let mut stream_opt = Some(stream);
        let mut read_stream: Option<tokio::net::tcp::OwnedReadHalf> = None;

        loop {
            let read_result = if let Some(ref mut rs) = read_stream {
                rs.read(&mut buffer).await
            } else if let Some(ref mut s) = stream_opt {
                s.read(&mut buffer).await
            } else {
                break;
            };
            
            match read_result {
                Ok(0) => {
                    info!("[DISCONNECTED] {} disconnected", addr);
                    break;
                }
                Ok(n) => {
                    let data = match String::from_utf8(buffer[..n].to_vec()) {
                        Ok(s) => s,
                        Err(e) => {
                            warn!("[ERROR] UTF-8 decode error: {}", e);
                            continue;
                        }
                    };

                    for message in data.split("\r\n") {
                        if message.is_empty() {
                            continue;
                        }
                        info!("[RECV] {}: {}", addr, message);

                        // Determine client type on first message
                        if first_message {
                            first_message = false;
                            
                            if message.contains("AA") {
                                // Controller login
                                if let Some(s) = stream_opt.take() {
                                    let (read_half, write_half) = s.into_split();
                                    let stream_arc = Arc::new(Mutex::new(write_half));
                                    let handler = Arc::new(Mutex::new(ControllerHandler::new(stream_arc)));
                                    controllers.lock().await.push(handler.clone());
                                    controller_handler = Some(handler);
                                    handler_type = Some(ClientType::Controller);
                                    read_stream = Some(read_half);
                                }
                            } else if message.contains("AP") {
                                // Pilot login
                                if let Some(s) = stream_opt.take() {
                                    let (read_half, write_half) = s.into_split();
                                    let stream_arc = Arc::new(Mutex::new(write_half));
                                    let handler = Arc::new(Mutex::new(PilotHandler::new(stream_arc)));
                                    pilots.lock().await.push(handler.clone());
                                    pilot_handler = Some(handler);
                                    handler_type = Some(ClientType::Pilot);
                                    read_stream = Some(read_half);
                                }
                            }
                        }

                        // Handle message based on client type
                        let status = match handler_type {
                            Some(ClientType::Controller) => {
                                if let Some(ref handler) = controller_handler {
                                    handler.lock().await.handle(message)?
                                } else {
                                    continue;
                                }
                            }
                            Some(ClientType::Pilot) => {
                                if let Some(ref handler) = pilot_handler {
                                    handler.lock().await.handle(message)?
                                } else {
                                    continue;
                                }
                            }
                            None => continue,
                        };

                        // Forward messages based on status
                        match status {
                            MessageStatus::Handled => {
                                // Special handling for flight plan queries
                                if message.contains("$CQ") && message.contains("FP") {
                                    Self::handle_flight_plan_query(
                                        message,
                                        &controllers,
                                        &pilots,
                                        controller_handler.as_ref(),
                                    ).await?;
                                }
                            }
                            MessageStatus::ForwardToControllers => {
                                // Forward to other controllers (not sender)
                                let sender_callsign = match handler_type {
                                    Some(ClientType::Controller) => {
                                        if let Some(ref h) = controller_handler {
                                            h.lock().await.callsign().to_string()
                                        } else {
                                            String::new()
                                        }
                                    }
                                    _ => String::new(),
                                };

                                Self::forward_to_controllers(message, &controllers, &sender_callsign).await?;
                            }
                            MessageStatus::ForwardToAllControllers => {
                                // Forward to all controllers
                                Self::forward_to_controllers(message, &controllers, "").await?;
                            }
                        }
                    }
                }
                Err(e) => {
                    error!("[ERROR] Read error: {}", e);
                    break;
                }
            }
        }

        // Cleanup: remove handler from lists
        if let Some(ClientType::Controller) = handler_type {
            if let Some(handler) = controller_handler {
                let mut ctrl_list = controllers.lock().await;
                ctrl_list.retain(|h| !Arc::ptr_eq(h, &handler));
            }
        } else if let Some(ClientType::Pilot) = handler_type {
            if let Some(handler) = pilot_handler {
                let mut pilot_list = pilots.lock().await;
                pilot_list.retain(|h| !Arc::ptr_eq(h, &handler));
            }
        }

        Ok(())
    }

    /// Handle flight plan query
    async fn handle_flight_plan_query(
        message: &str,
        _controllers: &Arc<Mutex<Vec<Arc<Mutex<ControllerHandler>>>>>,
        pilots: &Arc<Mutex<Vec<Arc<Mutex<PilotHandler>>>>>,
        requesting_controller: Option<&Arc<Mutex<ControllerHandler>>>,
    ) -> Result<()> {
        let parts: Vec<&str> = message.split(':').collect();
        if parts.len() < 4 {
            return Ok(());
        }

        let plane_callsign = parts[3];
        
        // Find the pilot
        let pilots_lock = pilots.lock().await;
        for pilot in pilots_lock.iter() {
            let pilot_guard = pilot.lock().await;
            if pilot_guard.callsign == plane_callsign {
                if let Some(controller) = requesting_controller {
                    // Send flight plan
                    if !pilot_guard.fp_message.is_empty() {
                        let fp_parts: Vec<&str> = pilot_guard.fp_message.iter()
                            .map(|s| s.as_str())
                            .collect();
                        controller.lock().await.send_message(&fp_parts).await?;
                    }

                    // Send squawk
                    let server_callsign = parts[1].to_string();
                    let cq_msg = format!("$CQ{}", server_callsign);
                    let squawk_parts = vec![
                        cq_msg.as_str(),
                        &server_callsign,
                        "BC",
                        plane_callsign,
                        &pilot_guard.squawk,
                    ];
                    controller.lock().await.send_message(&squawk_parts).await?;
                }
                break;
            }
        }

        Ok(())
    }

    /// Forward message to controllers
    async fn forward_to_controllers(
        message: &str,
        controllers: &Arc<Mutex<Vec<Arc<Mutex<ControllerHandler>>>>>,
        exclude_callsign: &str,
    ) -> Result<()> {
        let controllers_lock = controllers.lock().await;
        
        for controller in controllers_lock.iter() {
            let ctrl = controller.lock().await;
            if exclude_callsign.is_empty() || ctrl.callsign() != exclude_callsign {
                if let Err(e) = ctrl.send_message(&[message]).await {
                    warn!("[ERROR] Failed to send to controller {}: {}", ctrl.callsign(), e);
                }
            }
        }

        Ok(())
    }
}
