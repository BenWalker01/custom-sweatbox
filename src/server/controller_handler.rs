use anyhow::Result;
use tokio::io::AsyncWriteExt;
use tokio::net::tcp::OwnedWriteHalf;
use tokio::sync::Mutex;
use std::sync::Arc;

use super::message_handler::{MessageHandler, MessageStatus, ClientType, es_convert, parse_message};

/// Handler for controller connections
pub struct ControllerHandler {
    stream: Arc<Mutex<OwnedWriteHalf>>,
    pub callsign: String,
    server: String,
    cid: String,
    name: String,
    password: String,
    lat: String,
    lon: String,
    range: String,
    freq: String,
}

impl ControllerHandler {
    /// Create a new controller handler with the given stream
    pub fn new(stream: Arc<Mutex<OwnedWriteHalf>>) -> Self {
        Self {
            stream,
            callsign: String::new(),
            server: String::new(),
            cid: String::new(),
            name: String::new(),
            password: String::new(),
            lat: String::new(),
            lon: String::new(),
            range: String::new(),
            freq: String::new(),
        }
    }

    /// Send a message to this controller
    pub async fn send_message(&self, parts: &[&str]) -> Result<()> {
        let data = es_convert(parts);
        let mut stream = self.stream.lock().await;
        stream.write_all(&data).await?;
        Ok(())
    }
}

impl MessageHandler for ControllerHandler {
    fn handle(&mut self, message: &str) -> Result<MessageStatus> {
        let parts = parse_message(message);
        
        if parts.is_empty() {
            return Ok(MessageStatus::Handled);
        }

        // Handle controller login (#AA)
        if parts[0].starts_with("#AA") {
            if parts.len() >= 12 {
                self.callsign = parts[0][3..].to_string();
                self.server = parts.get(1).map(|s| s.to_string()).unwrap_or_default();
                self.name = parts.get(2).map(|s| s.to_string()).unwrap_or_default();
                self.cid = parts.get(3).map(|s| s.to_string()).unwrap_or_default();
                self.password = parts.get(4).map(|s| s.to_string()).unwrap_or_default();
                self.lat = parts.get(9).map(|s| s.to_string()).unwrap_or_default();
                self.lon = parts.get(10).map(|s| s.to_string()).unwrap_or_default();
                self.range = parts.get(11).map(|s| s.to_string()).unwrap_or_default();

                // Send welcome message
                let callsign = self.callsign.clone();
                let stream = self.stream.clone();
                tokio::spawn(async move {
                    let msg_parts = vec![
                        "#TMserver",
                        &callsign,
                        "Custom FSD server",
                    ];
                    let data = es_convert(&msg_parts);
                    if let Ok(_) = stream.lock().await.try_write(&data) {
                        // Message sent
                    }
                });
            }
            return Ok(MessageStatus::Handled);
        }

        // Handle position update (%)
        if parts[0].starts_with(&format!("%{}", self.callsign)) {
            if parts.len() >= 7 {
                self.freq = parts.get(1).map(|s| s.to_string()).unwrap_or_default();
                self.lat = parts.get(5).map(|s| s.to_string()).unwrap_or_default();
                self.lon = parts.get(6).map(|s| s.to_string()).unwrap_or_default();
            }
            return Ok(MessageStatus::ForwardToControllers);
        }

        // Handle query ($CQ)
        if parts[0].starts_with(&format!("$CQ{}", self.callsign)) {
            if parts.len() >= 3 {
                match parts[2].as_str() {
                    "IP" => {
                        // Respond to IP query
                        let server = self.server.clone();
                        let callsign = self.callsign.clone();
                        let stream = self.stream.clone();
                        tokio::spawn(async move {
                            let cr_msg = format!("$CR{}", server);
                            let msg_parts = vec![
                                cr_msg.as_str(),
                                &callsign,
                                "ATC",
                                "Y",
                                &callsign,
                            ];
                            let data = es_convert(&msg_parts);
                            if let Ok(_) = stream.lock().await.try_write(&data) {
                                // Message sent
                            }
                        });
                        
                        return Ok(MessageStatus::Handled);
                    }
                    "FP" => {
                        // Flight plan query - will be handled by server with pilot list
                        return Ok(MessageStatus::Handled);
                    }
                    _ => {}
                }
            }
            return Ok(MessageStatus::ForwardToControllers);
        }

        // Forward other messages to controllers
        Ok(MessageStatus::ForwardToControllers)
    }

    fn callsign(&self) -> &str {
        &self.callsign
    }

    fn client_type(&self) -> ClientType {
        ClientType::Controller
    }
}
