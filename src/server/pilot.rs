#[derive(Debug, Clone)]
pub struct PilotHandler {
    pub callsign: String,
    pub server: String,
    pub cid: String,
    pub name: String,
    pub password: String,
    pub lat: String,
    pub lon: String,
    pub range: String,
    pub freq: String,
    pub squawk: String,
    pub fp_message: Vec<String>,
}

impl PilotHandler {
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
            squawk: String::from("0000"),
            fp_message: Vec::new(),
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

        // Pilot login (#AA with 8 parameters)
        if parts[0].starts_with("#AA") && parts.len() < 12 {
            self.callsign = parts[0][3..].to_string();
            if parts.len() > 1 { self.server = parts[1].to_string(); }
            if parts.len() > 2 { self.cid = parts[2].to_string(); }
            if parts.len() > 3 { self.password = parts[3].to_string(); }
            if parts.len() > 7 { self.name = parts[7].to_string(); }
            
            return 0;
        }

        // Position update (@N)
        if parts[0].starts_with("@N") {
            if parts.len() > 2 {
                self.squawk = parts[2].to_string();
            }
            return 2; // Forward to all controllers
        }

        // Flight plan ($FP)
        if parts[0].starts_with("$FP") {
            self.fp_message = parts.iter().map(|s| s.to_string()).collect();
            return 2; // Forward to all controllers
        }

        // Default: forward to all controllers
        2
    }

    pub fn get_flight_plan_message(&self) -> String {
        self.fp_message.join(":")
    }
}
