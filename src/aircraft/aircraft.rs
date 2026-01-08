use crate::aircraft::flight_plan::FlightPlan;
use crate::utils::navigation::{FixDatabase, heading_from_to, position_bearing_distance, haversine_nm};

/// Aircraft phases of flight
#[derive(Debug, Clone, PartialEq)]
pub enum FlightPhase {
    OnGround,
    Departing,
    Climbing,
    Cruise,
    Descending,
    Approach,
    Landing,
}

/// Aircraft state
#[derive(Debug, Clone)]
pub struct Aircraft {
    pub callsign: String,
    pub aircraft_type: String,
    pub squawk: String,
    
    // Position
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: i32,      // feet
    pub heading: i32,       // degrees
    pub ground_speed: u32,  // knots
    
    // Flight plan
    pub flight_plan: FlightPlan,
    
    // Navigation
    pub route_fixes: Vec<String>,
    pub current_fix_index: usize,
    pub phase: FlightPhase,
    
    // Departure info
    pub departure_runway: String,
    pub departure_heading: i32,
    
    // Target values
    pub target_altitude: i32,
    pub target_heading: i32,
    pub target_speed: u32,
    
    // Time tracking
    pub spawn_time: std::time::Instant,
}

impl Aircraft {
    /// Create a new aircraft on the ground at departure airport
    pub fn new_departure(
        callsign: String,
        aircraft_type: String,
        squawk: String,
        departure: String,
        arrival: String,
        route: String,
        cruise_altitude: u32,
        runway: String,
        airport_coords: (f64, f64),
        runway_heading: i32,
    ) -> Self {
        let flight_plan = FlightPlan::new(
            aircraft_type.clone(),
            departure.clone(),
            arrival,
            cruise_altitude,
            route.clone(),
        );

        // Parse route to extract fixes
        let route_fixes = Self::parse_route(&route);
        
        // Extract SID altitude restriction (default to 6000 if not found)
        let sid_altitude = Self::extract_sid_altitude(&departure, &route);

        Self {
            callsign,
            aircraft_type,
            squawk,
            latitude: airport_coords.0,
            longitude: airport_coords.1,
            altitude: 0,
            heading: runway_heading,
            ground_speed: 0,
            flight_plan,
            route_fixes,
            current_fix_index: 0,
            phase: FlightPhase::OnGround,
            departure_runway: runway,
            departure_heading: runway_heading,
            target_altitude: sid_altitude,
            target_heading: runway_heading,
            target_speed: 250,
            spawn_time: std::time::Instant::now(),
        }
    }

    /// Placeholder for SID stop altitude - maybe just let UKCP set the tag and read from there??
    fn extract_sid_altitude(departure: &str, route: &str) -> i32 {
        // Common SID altitude restrictions by airport
        let default_restrictions = match departure {
            "EGSS" => 4000,  
            "EGGW" => 5000,  
            "EGLC" => 3000,
            "EGLL" => 6000,  
            "EGKK" => 4000,  
            _ => 6000,       
        };
        
        default_restrictions
    }
    
    /// Parse route string to extract fix names
    fn parse_route(route: &str) -> Vec<String> {
        let mut fixes = Vec::new();
        
        // Split by spaces and common route separators
        let parts: Vec<&str> = route.split(|c: char| c.is_whitespace() || c == '/')
            .filter(|s| !s.is_empty())
            .collect();
        
        for part in parts {
            // Skip SID/STAR notation (ends with numbers like CLN2E/22)
            if part.contains("/") {
                continue;
            }
            
            // Skip airway designators (start with letters followed by numbers)
            if part.len() >= 2 && part.chars().next().unwrap().is_alphabetic() {
                let has_digit = part.chars().any(|c| c.is_numeric());
                if has_digit && part.len() <= 5 {
                    // Likely an airway like P44, M197, Q295
                    continue;
                }
            }
            
            // Skip DCT
            if part == "DCT" {
                continue;
            }
            
            // This is likely a fix name
            if part.len() >= 3 && part.len() <= 6 && part.chars().all(|c| c.is_alphabetic()) {
                fixes.push(part.to_uppercase());
            }
        }
        
        fixes
    }

    /// Update aircraft position and state
    pub fn update(&mut self, delta_time: f64, fix_db: &FixDatabase, sim_config: &crate::config::SimulationConfig) {
        match self.phase {
            FlightPhase::OnGround => {
                // Wait a few seconds before starting takeoff
                if self.spawn_time.elapsed().as_secs() >= 5 {
                    self.phase = FlightPhase::Departing;
                    self.ground_speed = 10;
                }
            }
            
            FlightPhase::Departing => {
                // Accelerate on runway
                if self.ground_speed < 150 {
                    self.ground_speed += (50.0 * delta_time) as u32;
                } else {
                    // Rotate and start climbing
                    self.phase = FlightPhase::Climbing;
                    self.altitude = 50;
                    self.target_speed = 250;
                }
            }
            
            FlightPhase::Climbing => {
                // Realistic climb rate: 1500-2500 ft/min depending on altitude
                let climb_rate_fpm = if self.altitude < 10000 {
                    2000.0  // Higher rate at lower altitudes
                } else if self.altitude < 20000 {
                    1800.0  // Moderate rate
                } else {
                    1500.0  // Lower rate at higher altitudes
                };
                
                let climb_rate = (climb_rate_fpm / 60.0) * delta_time;  // Convert to ft/sec
                self.altitude += climb_rate as i32;
                
                // Accelerate to target speed
                if self.ground_speed < self.target_speed {
                    self.ground_speed += (10.0 * delta_time) as u32;
                }
                
                // Update speed restrictions and target altitude
                if self.altitude >= self.target_altitude && self.target_altitude < (self.flight_plan.cruise_altitude as i32 * 100) {
                    // Reached SID altitude, now climb to cruise
                    self.target_altitude = self.flight_plan.cruise_altitude as i32 * 100;
                    self.target_speed = 250;  // Maintain 250 until above 10000
                }
                
                if self.altitude > 10000 && self.target_speed < 300 {
                    self.target_speed = 300;
                }
                
                // Navigate to next fix
                self.navigate_to_next_fix(fix_db, delta_time, sim_config);
                
                // Check if reached final cruise altitude
                if self.altitude >= (self.flight_plan.cruise_altitude as i32 * 100) {
                    self.altitude = self.flight_plan.cruise_altitude as i32 * 100;
                    self.phase = FlightPhase::Cruise;
                    self.target_speed = self.flight_plan.cruise_speed;
                }
            }
            
            FlightPhase::Cruise => {
                // Maintain altitude and navigate
                self.navigate_to_next_fix(fix_db, delta_time, sim_config);
                
                // Accelerate to cruise speed
                if self.ground_speed < self.target_speed {
                    self.ground_speed += (5.0 * delta_time) as u32;
                }
            }
            
            _ => {
                // Other phases not implemented yet
            }
        }
        
        // Update position based on heading and speed
        self.update_position(delta_time);
    }

    /// Navigate towards the next fix
    fn navigate_to_next_fix(&mut self, fix_db: &FixDatabase, delta_time: f64, sim_config: &crate::config::SimulationConfig) {
        if self.current_fix_index >= self.route_fixes.len() {
            return;
        }
        
        let current_fix = &self.route_fixes[self.current_fix_index];
        
        if let Some((fix_lat, fix_lon)) = fix_db.get(current_fix) {
            // Calculate distance to fix
            let distance = haversine_nm(self.latitude, self.longitude, *fix_lat, *fix_lon);
            
            // If within 2 NM of fix, move to next fix
            if distance < 2.0 {
                self.current_fix_index += 1;
                
                if self.current_fix_index < self.route_fixes.len() {
                    let next_fix = &self.route_fixes[self.current_fix_index];
                    if let Some((next_lat, next_lon)) = fix_db.get(next_fix) {
                        self.target_heading = heading_from_to(self.latitude, self.longitude, *next_lat, *next_lon);
                    }
                }
            } else {
                // Turn towards fix
                let desired_heading = heading_from_to(self.latitude, self.longitude, *fix_lat, *fix_lon);
                self.turn_towards(desired_heading, delta_time, sim_config.turn_rate);
            }
        }
    }

    /// Turn towards a target heading
    fn turn_towards(&mut self, target: i32, delta_time: f64, turn_rate: f64) {
        let diff = ((target - self.heading + 540) % 360) - 180;
        
        if diff.abs() < 2 {
            self.heading = target;
        } else {
            let turn_amount = (turn_rate * delta_time) as i32;
            if diff > 0 {
                self.heading += turn_amount.min(diff);
            } else {
                self.heading -= turn_amount.min(diff.abs());
            }
            
            // Normalize heading
            self.heading = (self.heading + 360) % 360;
        }
    }

    /// Update position based on current heading and ground speed
    fn update_position(&mut self, delta_time: f64) {
        if self.ground_speed == 0 {
            return;
        }
        
        // Distance traveled in nautical miles
        let distance_nm = (self.ground_speed as f64 / 3600.0) * delta_time;
        
        // Update position
        let (new_lat, new_lon) = position_bearing_distance(
            self.latitude,
            self.longitude,
            self.heading as f64,
            distance_nm
        );
        
        self.latitude = new_lat;
        self.longitude = new_lon;
    }

    /// Format position for FSD protocol
    pub fn to_fsd_position(&self) -> String {
        // FSD format: @N:<callsign>:<squawk>:<rating>:<lat>:<lon>:<alt>:<groundspeed>:<heading>
        format!(
            "@N:{}:{}:1:{}:{}:{}:{}:{}",
            self.callsign,
            self.squawk,
            format!("{:.6}", self.latitude),
            format!("{:.6}", self.longitude),
            self.altitude,
            self.ground_speed,
            self.heading
        )
    }

    /// Get current fix being navigated to
    pub fn current_fix(&self) -> Option<&str> {
        self.route_fixes.get(self.current_fix_index).map(|s| s.as_str())
    }

    /// Check if aircraft has completed its route
    pub fn is_route_complete(&self) -> bool {
        self.current_fix_index >= self.route_fixes.len()
    }
}
