mod client;
mod controller;
mod pilot;
mod message;

pub use client::ClientHandler;
pub use message::ClientType;

use std::sync::Arc;
use tokio::sync::RwLock;
use tokio::net::{TcpListener, TcpStream};
use anyhow::Result;
use tracing::{info, error, warn};

const PORT: u16 = 6809;
const SERVER_ADDRESS: &str = "127.0.0.1";

pub type ClientList = Arc<RwLock<Vec<Arc<RwLock<ClientHandler>>>>>;

pub struct FsdServer {
    controllers: ClientList,
    pilots: ClientList,
}

impl FsdServer {
    pub fn new() -> Self {
        Self {
            controllers: Arc::new(RwLock::new(Vec::new())),
            pilots: Arc::new(RwLock::new(Vec::new())),
        }
    }

    pub async fn start(&self) -> Result<()> {
        let addr = format!("{}:{}", SERVER_ADDRESS, PORT);
        let listener = TcpListener::bind(&addr).await?;
        
        info!("[LISTENING] Server is listening on {}", addr);

        loop {
            match listener.accept().await {
                Ok((socket, addr)) => {
                    info!("[NEW CONNECTION] {} connected", addr);
                    
                    let controllers = Arc::clone(&self.controllers);
                    let pilots = Arc::clone(&self.pilots);
                    
                    tokio::spawn(async move {
                        if let Err(e) = handle_client(socket, addr, controllers, pilots).await {
                            error!("[ERROR] Error handling client {}: {}", addr, e);
                        }
                    });
                }
                Err(e) => {
                    error!("[ERROR] Failed to accept connection: {}", e);
                }
            }
        }
    }
}

async fn handle_client(
    socket: TcpStream,
    addr: std::net::SocketAddr,
    controllers: ClientList,
    pilots: ClientList,
) -> Result<()> {
    let client = Arc::new(RwLock::new(ClientHandler::new(socket)));
    
    // Run the client handler
    let _result = client.write().await.run().await;
    
    // Determine client type and add to appropriate list
    let client_type = client.read().await.client_type();
    
    match client_type {
        Some(ClientType::Controller) => {
            let callsign = client.read().await.callsign().to_string();
            info!("[CONTROLLER] {} identified as controller", callsign);
            
            let mut ctrl_list = controllers.write().await;
            ctrl_list.push(Arc::clone(&client));
            
            // Process messages
            loop {
                let msg = {
                    let mut c = client.write().await;
                    match c.receive_message().await {
                        Ok(Some(msg)) => msg,
                        Ok(None) => break,
                        Err(e) => {
                            warn!("[ERROR] Error receiving message: {}", e);
                            break;
                        }
                    }
                };
                
                let forward_mode = {
                    let mut c = client.write().await;
                    c.handle_message(&msg).await
                };
                
                // Forward message based on status
                match forward_mode {
                    1 => {
                        // Forward to other controllers
                        broadcast_to_controllers(&msg, &client, &controllers).await;
                    }
                    2 => {
                        // Forward to all controllers
                        broadcast_to_all_controllers(&msg, &controllers).await;
                    }
                    _ => {}
                }
            }
            
            // Remove from controllers list
            ctrl_list.retain(|c| !Arc::ptr_eq(c, &client));
        }
        Some(ClientType::Pilot) => {
            let callsign = client.read().await.callsign().to_string();
            info!("[PILOT] {} identified as pilot", callsign);
            
            let mut pilot_list = pilots.write().await;
            pilot_list.push(Arc::clone(&client));
            
            // Process messages
            loop {
                let msg = {
                    let mut c = client.write().await;
                    match c.receive_message().await {
                        Ok(Some(msg)) => msg,
                        Ok(None) => break,
                        Err(e) => {
                            warn!("[ERROR] Error receiving message: {}", e);
                            break;
                        }
                    }
                };
                
                let forward_mode = {
                    let mut c = client.write().await;
                    c.handle_message(&msg).await
                };
                
                // Forward message based on status
                match forward_mode {
                    2 => {
                        // Forward to all controllers
                        broadcast_to_all_controllers(&msg, &controllers).await;
                    }
                    _ => {}
                }
            }
            
            // Remove from pilots list
            pilot_list.retain(|p| !Arc::ptr_eq(p, &client));
        }
        None => {
            warn!("[DISCONNECTED] {} disconnected without identifying", addr);
        }
    }
    
    info!("[DISCONNECTED] {} disconnected", addr);
    Ok(())
}

async fn broadcast_to_controllers(
    msg: &str,
    sender: &Arc<RwLock<ClientHandler>>,
    controllers: &ClientList,
) {
    let sender_callsign = sender.read().await.callsign().to_string();
    let ctrl_list = controllers.read().await;
    
    for controller in ctrl_list.iter() {
        let callsign = controller.read().await.callsign().to_string();
        if callsign != sender_callsign {
            if let Err(e) = controller.write().await.send_message(msg).await {
                error!("[ERROR] Failed to send message to {}: {}", callsign, e);
            }
        }
    }
}

async fn broadcast_to_all_controllers(msg: &str, controllers: &ClientList) {
    let ctrl_list = controllers.read().await;
    
    for controller in ctrl_list.iter() {
        let callsign = controller.read().await.callsign().to_string();
        if let Err(e) = controller.write().await.send_message(msg).await {
            error!("[ERROR] Failed to send message to {}: {}", callsign, e);
        }
    }
}
