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

/// Lateral guidance mode
#[derive(Debug, Clone, PartialEq)]
pub enum LateralMode {
    /// Follow route fixes from the flight plan
    FlightPlan,
    /// Follow an assigned heading
    Heading,
    /// Fly a basic localizer/glideslope profile to runway threshold
    Ils,
}

/// Basic ILS guidance definition
#[derive(Debug, Clone)]
pub struct IlsGuidance {
    pub runway: String,
    pub threshold_lat: f64,
    pub threshold_lon: f64,
    pub runway_heading: i32,
    pub runway_elevation_ft: i32,
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
    pub lateral_mode: LateralMode,
    pub assumed_by: Option<String>,
    pub auto_climb_to_cruise: bool,
    pub ils_guidance: Option<IlsGuidance>,
    
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

        // Parse route to extract fixes (this gets the enroute portion)
        let enroute_fixes = Self::parse_route(&route);
        
        // Extract SID waypoints and prepend them to the route
        let sid_fixes = Self::extract_sid_waypoints(&departure, &route, &runway);
        let mut route_fixes = sid_fixes;
        
        // Add enroute fixes, but skip duplicates (e.g., if SID ends at CLN and route starts with CLN)
        for fix in enroute_fixes {
            if route_fixes.is_empty() || route_fixes.last() != Some(&fix) {
                route_fixes.push(fix);
            }
        }
        
        // Extract SID altitude restriction (default to 6000 if not found)
        let sid_altitude = Self::extract_sid_altitude(&departure, &route);

        tracing::info!("[AIRCRAFT] Creating {} with {} route fixes: {:?}", 
                      callsign, route_fixes.len(), route_fixes);

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
            lateral_mode: LateralMode::FlightPlan,
            assumed_by: None,
            auto_climb_to_cruise: true,
            ils_guidance: None,
            departure_runway: runway,
            departure_heading: runway_heading,
            target_altitude: sid_altitude,
            target_heading: runway_heading,
            target_speed: 250,
            spawn_time: std::time::Instant::now(),
        }
    }

    /// Placeholder for SID stop altitude - maybe just let UKCP set the tag and read from there??
    fn extract_sid_altitude(departure: &str, _route: &str) -> i32 {
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
    
    /// Extract SID waypoints from the SID file
    fn extract_sid_waypoints(departure: &str, route: &str, runway: &str) -> Vec<String> {
        // Extract SID name from route (e.g., "CLN2E/22" -> "CLN2E")
        let sid_name = if let Some(sid_part) = route.split_whitespace().next() {
            if sid_part.contains('/') {
                sid_part.split('/').next().unwrap_or("")
            } else {
                return Vec::new();
            }
        } else {
            return Vec::new();
        };
        
        // Try to load the SID file for this airport
        let sid_file = format!("data/Airports/{}/Sids.txt", departure);
        if let Ok(content) = std::fs::read_to_string(&sid_file) {
            for line in content.lines() {
                let line = line.trim();
                if line.is_empty() || line.starts_with(';') {
                    continue;
                }
                
                // Format: SID:ICAO:RUNWAY:SIDNAME:FIXES...
                let parts: Vec<&str> = line.split(':').collect();
                if parts.len() >= 5 && parts[0] == "SID" {
                    let file_runway = parts[2];
                    let file_sid_name = parts[3];
                    
                    // Match the SID name and runway
                    if file_sid_name == sid_name && file_runway == runway {
                        // Parse the waypoints
                        let fixes_str = parts[4];
                        let waypoints: Vec<String> = fixes_str
                            .split_whitespace()
                            .map(|s| s.to_uppercase())
                            .collect();
                        
                        tracing::debug!("[AIRCRAFT] Found SID {} for runway {}: {} waypoints", 
                                       sid_name, runway, waypoints.len());
                        return waypoints;
                    }
                }
            }
            tracing::warn!("[AIRCRAFT] SID {} not found for runway {} at {}", sid_name, runway, departure);
        } else {
            tracing::warn!("[AIRCRAFT] Could not read SID file: {}", sid_file);
        }
        
        Vec::new()
    }
    
    /// Parse route string to extract fix names
    fn parse_route(route: &str) -> Vec<String> {
        let mut fixes = Vec::new();
        
        // Split by spaces
        let parts: Vec<&str> = route.split(|c: char| c.is_whitespace())
            .filter(|s| !s.is_empty())
            .collect();
        
        for part in parts {
            // Skip SID/STAR notation with runway (e.g., CLN2E/22)
            if part.contains("/") {
                continue;
            }
            
            // Skip airway designators (start with letters followed by numbers, max 5 chars)
            if part.len() >= 2 && part.len() <= 5 {
                let chars: Vec<char> = part.chars().collect();
                if chars[0].is_alphabetic() {
                    let has_digit = chars.iter().any(|c| c.is_numeric());
                    let mostly_letters_then_numbers = 
                        chars.iter().take_while(|c| c.is_alphabetic()).count() <= 2 &&
                        has_digit;
                    
                    if mostly_letters_then_numbers {
                        // Likely an airway like P44, M197, Q295
                        continue;
                    }
                }
            }
            
            // Skip DCT
            if part == "DCT" {
                continue;
            }
            
            // This is likely a fix name (3-6 characters, all alphabetic)
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
                    tracing::info!("[{}] Starting takeoff roll", self.callsign);
                }
            }
            
            FlightPhase::Departing => {
                // Accelerate on runway
                if self.ground_speed < 150 {
                    self.ground_speed += (50.0 * delta_time) as u32;
                } else {
                    tracing::info!("[{}] Rotation speed reached, route_fixes.len()={}", 
                                  self.callsign, self.route_fixes.len());
                    // Rotate and start climbing
                    self.phase = FlightPhase::Climbing;
                    self.altitude = 50;
                    self.target_speed = 250;
                    
                    // Set initial heading towards first waypoint
                    if !self.route_fixes.is_empty() {
                        if let Some((fix_lat, fix_lon)) = fix_db.get(&self.route_fixes[0]) {
                            self.target_heading = heading_from_to(self.latitude, self.longitude, *fix_lat, *fix_lon);
                            self.heading = self.target_heading;
                            tracing::info!("[{}] Airborne, climbing to {} via {}", 
                                          self.callsign, self.route_fixes[0], self.route_fixes.join(" "));
                        } else {
                            tracing::warn!("[{}] First waypoint {} not found in nav database", 
                                          self.callsign, self.route_fixes[0]);
                        }
                    } else {
                        tracing::warn!("[{}] No route fixes available!", self.callsign);
                    }
                }
            }

            _ => {
                // If the aircraft is still on managed climb profile, move from SID cap to cruise.
                if self.auto_climb_to_cruise {
                    let cruise_altitude = self.flight_plan.cruise_altitude as i32 * 100;
                    if self.altitude >= self.target_altitude && self.target_altitude < cruise_altitude {
                        self.target_altitude = cruise_altitude;
                    }
                }

                self.update_vertical_profile(delta_time, sim_config);
                self.update_speed_profile(delta_time);

                match self.lateral_mode {
                    LateralMode::FlightPlan => self.navigate_to_next_fix(fix_db, delta_time, sim_config),
                    LateralMode::Heading => self.turn_towards(self.target_heading, delta_time, sim_config.turn_rate),
                    LateralMode::Ils => self.update_ils_guidance(delta_time, sim_config),
                }

                let cruise_altitude = self.flight_plan.cruise_altitude as i32 * 100;
                if self.altitude >= cruise_altitude && self.target_altitude >= cruise_altitude {
                    self.phase = FlightPhase::Cruise;
                } else if self.target_altitude > self.altitude {
                    self.phase = FlightPhase::Climbing;
                } else if self.target_altitude < self.altitude {
                    self.phase = FlightPhase::Descending;
                }
            }
        }
        
        // Update position based on heading and speed
        self.update_position(delta_time);
    }

    fn update_vertical_profile(&mut self, delta_time: f64, sim_config: &crate::config::SimulationConfig) {
        let altitude_delta = self.target_altitude - self.altitude;

        if altitude_delta.abs() <= 50 {
            self.altitude = self.target_altitude;
            return;
        }

        if altitude_delta > 0 {
            let climb_fpm = if self.altitude < 10000 {
                sim_config.climb_rate
            } else {
                sim_config.climb_rate * 0.8
            };

            let step = ((climb_fpm / 60.0) * delta_time).max(1.0) as i32;
            self.altitude += step.min(altitude_delta);
        } else {
            let descent_fpm = if altitude_delta.abs() > 6000 {
                sim_config.high_descent_rate.abs()
            } else {
                sim_config.descent_rate.abs()
            };

            let step = ((descent_fpm / 60.0) * delta_time).max(1.0) as i32;
            self.altitude -= step.min(altitude_delta.abs());
        }

        if self.altitude < 0 {
            self.altitude = 0;
        }
    }

    fn update_speed_profile(&mut self, delta_time: f64) {
        if self.ground_speed == self.target_speed {
            return;
        }

        let speed_step = (8.0 * delta_time).max(1.0) as u32;

        if self.ground_speed < self.target_speed {
            self.ground_speed = (self.ground_speed + speed_step).min(self.target_speed);
        } else {
            self.ground_speed = self.ground_speed.saturating_sub(speed_step).max(self.target_speed);
        }
    }

    fn update_ils_guidance(&mut self, delta_time: f64, sim_config: &crate::config::SimulationConfig) {
        let Some(ils) = self.ils_guidance.as_ref() else {
            self.lateral_mode = LateralMode::FlightPlan;
            return;
        };

        let threshold_lat = ils.threshold_lat;
        let threshold_lon = ils.threshold_lon;
        let runway_heading = ils.runway_heading;
        let runway_elevation_ft = ils.runway_elevation_ft;

        let distance_nm = haversine_nm(self.latitude, self.longitude, threshold_lat, threshold_lon);

        if distance_nm > 8.0 {
            let intercept_heading = heading_from_to(self.latitude, self.longitude, threshold_lat, threshold_lon);
            self.turn_towards(intercept_heading, delta_time, sim_config.turn_rate);
        } else {
            self.turn_towards(runway_heading, delta_time, sim_config.turn_rate);
        }

        let glideslope_altitude =
            ((distance_nm * 6076.0 * 3.0_f64.to_radians().tan()).round() as i32) + runway_elevation_ft;

        if self.altitude > glideslope_altitude + 100 {
            self.target_altitude = glideslope_altitude.max(runway_elevation_ft);
        }

        if distance_nm < 10.0 && self.target_speed > 180 {
            self.target_speed = 180;
        }

        if distance_nm < 4.0 && self.target_speed > 150 {
            self.target_speed = 150;
        }

        if distance_nm < 6.0 {
            self.phase = FlightPhase::Approach;
        }

        if distance_nm < 1.0 {
            self.target_altitude = runway_elevation_ft;
            self.phase = FlightPhase::Landing;
        }
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
            
            // Calculate required heading to fix
            let required_heading = heading_from_to(self.latitude, self.longitude, *fix_lat, *fix_lon);
            
            // If within 0.5 NM of fix, move to next fix
            if distance < 0.5 {
                self.current_fix_index += 1;
                
                if self.current_fix_index < self.route_fixes.len() {
                    let next_fix = &self.route_fixes[self.current_fix_index];
                    if let Some((next_lat, next_lon)) = fix_db.get(next_fix) {
                        self.target_heading = heading_from_to(self.latitude, self.longitude, *next_lat, *next_lon);
                        tracing::info!("[{}] Passed {}, turning to next waypoint: {}", 
                                      self.callsign, current_fix, next_fix);
                    }
                }
            }
            
            // Always turn towards the current fix (whether we just updated it or not)
            self.turn_towards(required_heading, delta_time, sim_config.turn_rate);
        }
    }

    /// Turn towards a target heading
    fn turn_towards(&mut self, target: i32, delta_time: f64, turn_rate: f64) {
        let diff = ((target - self.heading + 540) % 360) - 180;
        
        if diff.abs() < 2 {
            self.heading = target;
        } else {
            // Calculate turn amount as float first, then convert to int (fixes rounding to 0)
            let turn_amount_f = turn_rate * delta_time;
            let turn_amount = turn_amount_f.max(1.0) as i32;  // Ensure at least 1 degree per update
            
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

    /// Mark which controller currently assumes this aircraft.
    pub fn set_assumed_by(&mut self, controller: Option<String>) {
        self.assumed_by = controller;
    }

    /// Apply a heading assignment and switch to heading mode.
    pub fn assign_heading(&mut self, heading: i32) {
        self.target_heading = (heading % 360 + 360) % 360;
        self.lateral_mode = LateralMode::Heading;
        self.ils_guidance = None;
    }

    /// Apply a speed assignment.
    pub fn assign_speed(&mut self, speed: u32) {
        self.target_speed = speed.clamp(120, 500);
    }

    /// Apply an altitude assignment.
    pub fn assign_altitude(&mut self, altitude: i32) {
        self.target_altitude = altitude.max(0);
        self.auto_climb_to_cruise = false;

        if self.target_altitude > self.altitude {
            self.phase = FlightPhase::Climbing;
        } else if self.target_altitude < self.altitude {
            self.phase = FlightPhase::Descending;
        }
    }

    /// Assign an ILS approach using runway threshold and heading.
    pub fn assign_ils(
        &mut self,
        runway: String,
        threshold: (f64, f64),
        runway_heading: i32,
        runway_elevation_ft: i32,
    ) {
        self.ils_guidance = Some(IlsGuidance {
            runway,
            threshold_lat: threshold.0,
            threshold_lon: threshold.1,
            runway_heading,
            runway_elevation_ft,
        });

        self.auto_climb_to_cruise = false;
        self.lateral_mode = LateralMode::Ils;
        self.phase = FlightPhase::Approach;

        if self.target_speed > 200 {
            self.target_speed = 200;
        }
    }

    /// Return to route-based navigation mode.
    pub fn resume_navigation(&mut self) {
        self.lateral_mode = LateralMode::FlightPlan;
        self.ils_guidance = None;
    }

    /// Proceed direct to a fix. If it is not in the remaining route but exists in
    /// nav data, insert it as the immediate next waypoint.
    pub fn direct_to_fix(&mut self, fix: &str, fix_db: &FixDatabase) -> bool {
        let target_fix = fix.to_uppercase();

        if self.current_fix_index >= self.route_fixes.len() {
            if fix_db.contains_key(&target_fix) {
                self.route_fixes.push(target_fix);
                if !self.route_fixes.is_empty() {
                    self.current_fix_index = self.route_fixes.len() - 1;
                }
                self.lateral_mode = LateralMode::FlightPlan;
                self.ils_guidance = None;
                return true;
            }

            return false;
        }
        let search_slice = &self.route_fixes[self.current_fix_index..];

        if let Some(offset) = search_slice.iter().position(|f| f == &target_fix) {
            self.current_fix_index += offset;
            self.lateral_mode = LateralMode::FlightPlan;
            self.ils_guidance = None;
            true
        } else if fix_db.contains_key(&target_fix) {
            self.route_fixes.insert(self.current_fix_index, target_fix);
            self.lateral_mode = LateralMode::FlightPlan;
            self.ils_guidance = None;
            true
        } else {
            false
        }
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
