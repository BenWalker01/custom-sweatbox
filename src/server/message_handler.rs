use anyhow::Result;

/// Represents the type of client connection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClientType {
    Controller,
    Pilot,
}

/// Message handling status returned after processing
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageStatus {
    /// Message handled, do not forward (0 in Python)
    Handled,
    /// Forward to other controllers (1 in Python)
    ForwardToControllers,
    /// Forward to all controllers (2 in Python)
    ForwardToAllControllers,
}

/// Trait for handling FSD protocol messages
pub trait MessageHandler: Send + Sync {
    /// Handle an incoming message
    fn handle(&mut self, message: &str) -> Result<MessageStatus>;
    
    /// Get the callsign of this client
    fn callsign(&self) -> &str;
    
    /// Get the client type
    fn client_type(&self) -> ClientType;
}

/// Convert arguments to FSD format (colon-separated with \r\n)
pub fn es_convert(parts: &[&str]) -> Vec<u8> {
    let mut result = parts.join(":");
    result.push_str("\r\n");
    result.into_bytes()
}

/// Split an FSD message into parts
pub fn parse_message(message: &str) -> Vec<String> {
    message.split(':').map(|s| s.to_string()).collect()
}
