use anyhow::Result;
use tokio::net::tcp::OwnedWriteHalf;
use tokio::sync::Mutex;
use std::sync::Arc;

use super::message_handler::{MessageHandler, MessageStatus, ClientType, parse_message};

/// Handler for pilot connections
pub struct PilotHandler {
    #[allow(dead_code)]
    stream: Arc<Mutex<OwnedWriteHalf>>,
    pub callsign: String,
    server: String,
    cid: String,
    name: String,
    password: String,
    #[allow(dead_code)]
    lat: String,
    #[allow(dead_code)]
    lon: String,
    pub squawk: String,
    pub fp_message: Vec<String>,
}

impl PilotHandler {
    /// Create a new pilot handler with the given stream
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
            squawk: "0000".to_string(),
            fp_message: Vec::new(),
        }
    }
}

impl MessageHandler for PilotHandler {
    fn handle(&mut self, message: &str) -> Result<MessageStatus> {
        let parts = parse_message(message);
        
        if parts.is_empty() {
            return Ok(MessageStatus::Handled);
        }

        // Handle pilot login (#AP)
        if parts[0].starts_with("#AP") {
            if parts.len() >= 8 {
                self.callsign = parts[0][3..].to_string();
                self.server = parts.get(1).map(|s| s.to_string()).unwrap_or_default();
                self.cid = parts.get(2).map(|s| s.to_string()).unwrap_or_default();
                self.password = parts.get(3).map(|s| s.to_string()).unwrap_or_default();
                self.name = parts.get(7).map(|s| s.to_string()).unwrap_or_default();
            }
            return Ok(MessageStatus::Handled);
        }

        // Handle squawk assignment (@N)
        if parts[0].starts_with("@N") {
            if parts.len() >= 3 {
                self.squawk = parts.get(2).map(|s| s.to_string()).unwrap_or_default();
            }
            return Ok(MessageStatus::ForwardToAllControllers);
        }

        // Handle flight plan ($FP)
        if parts[0].starts_with("$FP") {
            self.fp_message = parts;
            return Ok(MessageStatus::ForwardToAllControllers);
        }

        // Forward all other pilot messages to all controllers
        Ok(MessageStatus::ForwardToAllControllers)
    }

    fn callsign(&self) -> &str {
        &self.callsign
    }

    fn client_type(&self) -> ClientType {
        ClientType::Pilot
    }
}
