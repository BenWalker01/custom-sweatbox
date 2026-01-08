use anyhow::{Result, Context};
use tokio::net::TcpStream;
use tokio::io::{AsyncWriteExt};
use tracing::{info, debug};

/// AI Pilot client that connects to the FSD server
pub struct AiPilot {
    stream: Option<TcpStream>,
    callsign: String,
    cid: String,
}

impl AiPilot {
    /// Create a new AI pilot
    pub fn new(callsign: String) -> Self {
        Self {
            stream: None,
            callsign,
            cid: "1000001".to_string(),
        }
    }

    /// Connect to the FSD server
    pub async fn connect(&mut self, server_addr: &str) -> Result<()> {
        debug!("[AI PILOT] {} connecting to FSD server at {}", self.callsign, server_addr);
        
        let stream = TcpStream::connect(server_addr)
            .await
            .context(format!("Failed to connect to {}", server_addr))?;
        
        self.stream = Some(stream);
        
        debug!("[AI PILOT] {} connected to FSD server", self.callsign);
        Ok(())
    }

    /// Login to the FSD server as a pilot
    pub async fn login(&mut self, aircraft_type: &str, squawk: &str) -> Result<()> {
        if self.stream.is_none() {
            return Err(anyhow::anyhow!("Not connected to server"));
        }

        info!("[AI PILOT] {} logging in as {}", self.callsign, aircraft_type);

        // FSD pilot login format: #AP<callsign>:<server>:<cid>:<password>:<rating>:<protocol>:<simulator>:<realname>
        let login_message = format!(
            "#AP{}:SERVER:{}:123456:1:100:1:AI Pilot\r\n",
            self.callsign,
            self.cid
        );

        self.send_raw(&login_message).await?;
        
        info!("[AI PILOT] Login message sent for {}: {}", self.callsign, login_message.trim());

        // Wait for server response
        tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;

        // Send initial squawk
        let squawk_message = format!("@S:{}:{}\r\n", self.callsign, squawk);
        self.send_raw(&squawk_message).await?;
        info!("[AI PILOT] Squawk set for {}: {}", self.callsign, squawk);

        Ok(())
    }

    /// Send a position update
    pub async fn send_position(&mut self, 
        lat: f64, 
        lon: f64, 
        altitude: i32, 
        ground_speed: u32, 
        heading: i32,
        squawk: &str
    ) -> Result<()> {
        // FSD pilot position format: @N:<callsign>:<squawk>:<rating>:<lat>:<lon>:<alt>:<groundspeed>:<pbh>:<flags>
        let position_message = format!(
            "@N:{}:{}:1:{:.6}:{:.6}:{}:{}:0:0:0\r\n",
            self.callsign,
            squawk,
            lat,
            lon,
            altitude,
            ground_speed
        );

        self.send_raw(&position_message).await?;
        debug!("[AI PILOT] Position update sent for {}: lat={:.6}, lon={:.6}, alt={}, spd={}, hdg={}", 
               self.callsign, lat, lon, altitude, ground_speed, heading);
        
        Ok(())
    }

    /// Send a flight plan
    pub async fn send_flight_plan(&mut self, flight_plan: &str) -> Result<()> {
        let fp_message = format!("$FP{}:{}\r\n", self.callsign, flight_plan);
        self.send_raw(&fp_message).await?;
        info!("[AI PILOT] Flight plan filed for {}: {}", self.callsign, flight_plan);
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

    /// Disconnect from the server
    pub async fn disconnect(&mut self) -> Result<()> {
        if let Some(mut stream) = self.stream.take() {
            info!("[AI PILOT] Disconnecting {}", self.callsign);
            
            // Send disconnect message
            let disconnect_msg = format!("#DP{}\r\n", self.callsign);
            stream.write_all(disconnect_msg.as_bytes()).await?;
            stream.flush().await?;
            
            stream.shutdown().await?;
        }
        
        Ok(())
    }

    /// Get the callsign
    pub fn callsign(&self) -> &str {
        &self.callsign
    }
}
