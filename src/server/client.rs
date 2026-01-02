use tokio::net::TcpStream;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use anyhow::{Result, Context};
use std::io;

use super::message::{FsdMessage, ClientType};
use super::controller::ControllerHandler;
use super::pilot::PilotHandler;

enum HandlerType {
    Controller(ControllerHandler),
    Pilot(PilotHandler),
}

pub struct ClientHandler {
    socket: TcpStream,
    handler: Option<HandlerType>,
    buffer: Vec<u8>,
}

impl ClientHandler {
    pub fn new(socket: TcpStream) -> Self {
        Self {
            socket,
            handler: None,
            buffer: Vec::new(),
        }
    }

    pub async fn run(&mut self) -> Result<()> {
        if let Some(msg) = self.receive_message().await? {
            if msg.contains("#AA") {
                // Both controllers and pilots use #AA, distinguish by parameter count
                let parts: Vec<&str> = msg.split(':').collect();
                
                // Controller: #AA + callsign + 10 more params (12 total)
                // Pilot: #AA + callsign + 7 more params (9 total)
                if parts.len() >= 12 {
                    // Controller login
                    let mut controller = ControllerHandler::new();
                    controller.handle(&msg);
                    
                    let response = FsdMessage::encode(&[
                        "#TMserver",
                        &controller.callsign,
                        "Custom FSD Server"
                    ]);
                    self.send_message(&response).await?;
                    
                    self.handler = Some(HandlerType::Controller(controller));
                } else {
                    // Pilot login
                    let mut pilot = PilotHandler::new();
                    pilot.handle(&msg);
                    self.handler = Some(HandlerType::Pilot(pilot));
                }
            }
        }
        
        Ok(())
    }

    pub async fn receive_message(&mut self) -> Result<Option<String>> {
        loop {
            // Check if we have a complete message in buffer
            if let Some(pos) = self.find_message_end() {
                let msg_bytes = self.buffer.drain(..pos + 2).collect::<Vec<u8>>();
                let msg = String::from_utf8_lossy(&msg_bytes[..msg_bytes.len() - 2]).to_string();
                return Ok(Some(msg));
            }

            // Read more data
            let mut temp_buf = vec![0u8; 262144];
            
            match self.socket.read(&mut temp_buf).await {
                Ok(0) => return Ok(None), // Connection closed
                Ok(n) => {
                    self.buffer.extend_from_slice(&temp_buf[..n]);
                    // Continue loop to check for complete message
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                    return Ok(None);
                }
                Err(e) => return Err(e.into()),
            }
        }
    }

    fn find_message_end(&self) -> Option<usize> {
        for i in 0..self.buffer.len().saturating_sub(1) {
            if self.buffer[i] == b'\r' && self.buffer[i + 1] == b'\n' {
                return Some(i);
            }
        }
        None
    }

    pub async fn send_message(&mut self, msg: &str) -> Result<()> {
        let data = if msg.ends_with("\r\n") {
            msg.to_string()
        } else {
            format!("{}\r\n", msg)
        };
        
        self.socket.write_all(data.as_bytes()).await
            .context("Failed to write to socket")?;
        Ok(())
    }

    pub async fn handle_message(&mut self, msg: &str) -> i32 {
        match &mut self.handler {
            Some(HandlerType::Controller(controller)) => {
                controller.handle(msg)
            }
            Some(HandlerType::Pilot(pilot)) => {
                pilot.handle(msg)
            }
            None => 0,
        }
    }

    pub fn client_type(&self) -> Option<ClientType> {
        match &self.handler {
            Some(HandlerType::Controller(_)) => Some(ClientType::Controller),
            Some(HandlerType::Pilot(_)) => Some(ClientType::Pilot),
            None => None,
        }
    }

    pub fn callsign(&self) -> &str {
        match &self.handler {
            Some(HandlerType::Controller(c)) => &c.callsign,
            Some(HandlerType::Pilot(p)) => &p.callsign,
            None => "",
        }
    }

    pub fn get_controller(&self) -> Option<&ControllerHandler> {
        match &self.handler {
            Some(HandlerType::Controller(c)) => Some(c),
            _ => None,
        }
    }

    pub fn get_pilot(&self) -> Option<&PilotHandler> {
        match &self.handler {
            Some(HandlerType::Pilot(p)) => Some(p),
            _ => None,
        }
    }
}
