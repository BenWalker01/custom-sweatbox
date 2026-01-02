use super::message::FsdMessage;

#[derive(Debug, Clone)]
pub struct ControllerHandler {
    pub callsign: String,
    pub server: String,
    pub cid: String,
    pub name: String,
    pub password: String,
    pub lat: String,
    pub lon: String,
    pub range: String,
    pub freq: String,
}

impl ControllerHandler {
    pub fn new() -> Self {
        Self {
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

    /// Handle a message and return status code:
    /// 0 = don't forward
    /// 1 = forward to other controllers
    /// 2 = forward to all controllers
    pub fn handle(&mut self, message: &str) -> i32 {
        let parts: Vec<&str> = message.split(':').collect();
        
        if parts.is_empty() {
            return 0;
        }

        // Controller login (#AA)
        if parts[0].starts_with("#AA") {
            self.callsign = parts[0][3..].to_string();
            if parts.len() > 1 { self.server = parts[1].to_string(); }
            if parts.len() > 2 { self.name = parts[2].to_string(); }
            if parts.len() > 3 { self.cid = parts[3].to_string(); }
            if parts.len() > 4 { self.password = parts[4].to_string(); }
            if parts.len() > 9 { self.lat = parts[9].to_string(); }
            if parts.len() > 10 { self.lon = parts[10].to_string(); }
            if parts.len() > 11 { self.range = parts[11].to_string(); }
            
            return 0;
        }

        // Controller position update (%)
        if parts[0].starts_with('%') && parts[0][1..].starts_with(&self.callsign) {
            if parts.len() > 1 { self.freq = parts[1].to_string(); }
            if parts.len() > 5 { self.lat = parts[5].to_string(); }
            if parts.len() > 6 { self.lon = parts[6].to_string(); }
            
            return 1; // Forward to other controllers
        }

        // Controller query ($CQ)
        if parts[0].starts_with("$CQ") && parts[0][3..].starts_with(&self.callsign) {
            if parts.len() > 2 {
                match parts[2] {
                    "IP" => {
                        // This should send a response, but we can't do that here
                        // The caller will need to handle this
                        return 0;
                    }
                    "FP" => {
                        // Flight plan query - caller needs to handle
                        return 0;
                    }
                    _ => return 1,
                }
            }
            return 1;
        }

        // Default: forward to other controllers
        1
    }

    pub fn create_ip_response(&self) -> String {
        FsdMessage::encode(&[
            &format!("$CR{}", self.server),
            &self.callsign,
            "ATC",
            "Y",
            &self.callsign,
        ])
    }

    pub fn create_squawk_response(&self, plane_callsign: &str, squawk: &str) -> String {
        FsdMessage::encode(&[
            &format!("$CQ{}", self.server),
            &self.callsign,
            "BC",
            plane_callsign,
            squawk,
        ])
    }
}
