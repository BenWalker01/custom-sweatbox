use anyhow::{Result, Context};
use tokio::net::TcpStream;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::mpsc;
use tracing::{info, debug, warn, error};

/// AI Controller client that connects to the FSD server
pub struct AiController {
    stream: Option<TcpStream>,
    tx: Option<mpsc::UnboundedSender<String>>,
    callsign: String,
    freq: String,
    name: String,
    cid: String,
    password: String,
    latitude: f64,
    longitude: f64,
    range: u32,
}

impl AiController {
    /// Create a new AI controller
    pub fn new(
        callsign: String,
        freq: String,
        latitude: f64,
        longitude: f64,
        range: u32,
    ) -> Self {
        Self {
            stream: None,
            tx: None,
            callsign,
            freq,
            name: "AI Controller".to_string(),
            cid: "1000000".to_string(),
            password: "123456".to_string(),
            latitude,
            longitude,
            range,
        }
    }

    /// Connect to the FSD server
    pub async fn connect(&mut self, server_addr: &str) -> Result<()> {
        info!("[AI CONTROLLER] Connecting to FSD server at {}", server_addr);
        
        let stream = TcpStream::connect(server_addr)
            .await
            .context(format!("Failed to connect to {}", server_addr))?;
        
        self.stream = Some(stream);
        
        info!("[AI CONTROLLER] Connected to FSD server");
        Ok(())
    }

    /// Login to the FSD server as a controller
    pub async fn login(&mut self) -> Result<()> {
        if self.stream.is_none() {
            return Err(anyhow::anyhow!("Not connected to server"));
        }

        info!("[AI CONTROLLER] Logging in as {}", self.callsign);

        // FSD controller login format: #AA<callsign>:<server>:<name>:<cid>:<password>:<rating>:<protocol>:<simulator>:<callsign>:<lat>:<lon>:<range>
        let login_message = format!(
            "#AA{}:SERVER:{}:{}:{}:5:100:1:100:{}:{}:{}\r\n",
            self.callsign,
            self.name,
            self.cid,
            self.password,
            self.latitude,
            self.longitude,
            self.range
        );

        self.send_raw(&login_message).await?;
        
        info!("[AI CONTROLLER] Login message sent for {}", self.callsign);

        // Wait for server response
        tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;

        // Send initial position update
        self.send_position_update().await?;

        info!("[AI CONTROLLER] {} logged in successfully on {}", self.callsign, self.freq);
        
        Ok(())
    }

    /// Send a position update
    pub async fn send_position_update(&mut self) -> Result<()> {
        // FSD controller position format: %<callsign>:<frequency>:<facilitytype>:<visrange>:<rating>:<lat>:<lon>:<elevation>
        let position_message = format!(
            "%{}:{}:4:{}:5:{}:{}:0\r\n",
            self.callsign,
            self.freq,
            self.range,
            self.latitude,
            self.longitude
        );

        self.send_raw(&position_message).await?;
        debug!("[AI CONTROLLER] Position update sent for {}", self.callsign);
        
        Ok(())
    }

    /// Send an IP query (capabilities)
    pub async fn send_ip_query(&mut self) -> Result<()> {
        let query = format!("$CQSERVER:{}:IP\r\n", self.callsign);
        self.send_raw(&query).await?;
        debug!("[AI CONTROLLER] IP query sent for {}", self.callsign);
        Ok(())
    }

    /// Send a raw message to the server
    async fn send_raw(&mut self, message: &str) -> Result<()> {
        if let Some(stream) = &mut self.stream {
            stream.write_all(message.as_bytes()).await?;
            stream.flush().await?;
            Ok(())
        } else {
            Err(anyhow::anyhow!("Not connected to server"))
        }
    }

    /// Start listening for messages from the server
    pub async fn start_message_loop(&mut self) -> Result<()> {
        if self.stream.is_none() {
            return Err(anyhow::anyhow!("Not connected to server"));
        }

        let stream = self.stream.take().unwrap();
        let (mut read_half, mut write_half) = stream.into_split();
        
        let callsign = self.callsign.clone();
        let callsign_write = callsign.clone();
        let callsign_periodic = callsign.clone();
        let freq = self.freq.clone();
        let latitude = self.latitude;
        let longitude = self.longitude;
        let range = self.range;

        // Create channel for sending messages
        let (tx, mut rx) = mpsc::unbounded_channel::<String>();
        self.tx = Some(tx.clone());

        // Spawn a task to handle outgoing messages
        tokio::spawn(async move {
            while let Some(message) = rx.recv().await {
                if let Err(e) = write_half.write_all(message.as_bytes()).await {
                    error!("[AI CONTROLLER] Failed to send message: {}", e);
                    break;
                }
                if let Err(e) = write_half.flush().await {
                    error!("[AI CONTROLLER] Failed to flush: {}", e);
                    break;
                }
            }
            info!("[AI CONTROLLER] Write loop ended for {}", callsign_write);
        });

        // Spawn a task to handle incoming messages
        tokio::spawn(async move {
            let mut buffer = vec![0u8; 8192];
            
            loop {
                match read_half.read(&mut buffer).await {
                    Ok(0) => {
                        warn!("[AI CONTROLLER] {} - Server disconnected", callsign);
                        break;
                    }
                    Ok(n) => {
                        if let Ok(data) = String::from_utf8(buffer[..n].to_vec()) {
                            for message in data.split("\r\n") {
                                if message.is_empty() {
                                    continue;
                                }
                                debug!("[AI CONTROLLER] {} received: {}", callsign, message);
                            }
                        }
                    }
                    Err(e) => {
                        error!("[AI CONTROLLER] {} - Read error: {}", callsign, e);
                        break;
                    }
                }
            }
            info!("[AI CONTROLLER] Read loop ended for {}", callsign);
        });

        // Spawn a task to send periodic position updates
        let tx_periodic = tx;
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(tokio::time::Duration::from_secs(30));
            
            loop {
                interval.tick().await;
                
                let position_message = format!(
                    "%{}:{}:4:{}:5:{}:{}:0\r\n",
                    callsign_periodic, freq, range, latitude, longitude
                );
                
                if tx_periodic.send(position_message).is_err() {
                    debug!("[AI CONTROLLER] Position update channel closed for {}", callsign_periodic);
                    break;
                }
            }
        });

        Ok(())
    }

    /// Disconnect from the server
    pub async fn disconnect(&mut self) -> Result<()> {
        info!("[AI CONTROLLER] Disconnecting {}", self.callsign);
        
        // Send disconnect message through channel if available
        if let Some(tx) = &self.tx {
            let disconnect_msg = format!("#DA{}:SERVER\r\n", self.callsign);
            let _ = tx.send(disconnect_msg);
            
            // Give it time to send
            tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
        }
        
        // Drop the channel sender to close the write loop
        self.tx = None;
        
        Ok(())
    }

    /// Get the callsign
    pub fn callsign(&self) -> &str {
        &self.callsign
    }

    /// Get the frequency
    pub fn frequency(&self) -> &str {
        &self.freq
    }
}

impl Drop for AiController {
    fn drop(&mut self) {
        if self.tx.is_some() {
            warn!("[AI CONTROLLER] {} dropped without proper disconnect", self.callsign);
        }
    }
}
