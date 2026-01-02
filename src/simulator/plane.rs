use std::time::{SystemTime, UNIX_EPOCH};
use super::flight_plan::FlightPlan;
use super::plane_mode::PlaneMode;

const TURN_RATE: f64 = 2.0; // degrees per second
const TAXI_SPEED: f64 = 15.0; // knots
const PUSH_SPEED: f64 = 5.0; // knots
const CLIMB_RATE: f64 = 2500.0; // feet per minute
const DESCENT_RATE: f64 = -2000.0; // feet per minute
const HIGH_DESCENT_RATE: f64 = -3000.0; // feet per minute

#[derive(Debug, Clone)]
pub struct Plane {
    // Basic identification
    pub callsign: String,
    pub squawk: u32,
    
    // Position and movement
    pub altitude: f64,      // feet
    pub heading: f64,       // 0-360 degrees
    pub speed: f64,         // knots (IAS)
    pub lat: f64,           // decimal degrees
    pub lon: f64,           // decimal degrees
    pub vert_speed: f64,    // feet per minute
    
    // Mode and state
    pub mode: PlaneMode,
    pub flight_plan: FlightPlan,
    
    // Target values
    pub target_speed: f64,
    pub target_altitude: f64,
    pub target_heading: f64,
    pub turn_dir: Option<TurnDirection>,
    
    // Hold pattern
    pub hold_fix: Option<String>,
    pub hold_start_time: Option<f64>,
    
    // Aircraft characteristics
    pub aircraft_type: String,
    pub vref: f64,
    
    // ILS approach
    pub cleared_ils: Option<ILSClearance>,
    pub runway_heading: Option<f64>,
    pub old_alt: Option<f64>,
    pub old_head: Option<f64>,
    
    // Ground operations
    pub ground_position: Option<String>,
    pub ground_route: Option<Vec<GroundPoint>>,
    pub stand: Option<String>,
    pub first_ground_position: Option<(f64, f64)>,
    
    // Controller handoff
    pub currently_with_data: Option<ControllerData>,
    pub first_controller: Option<String>,
    
    // Altitude management
    pub vert_mode: i32, // -1: desc, 0: level, 1: climb
    pub lvl_coords: Option<(f64, f64)>,
    pub die_on_reaching_2k: bool,
    
    // Time tracking
    last_time: f64,
}

#[derive(Debug, Clone, Copy)]
pub enum TurnDirection {
    Left,
    Right,
}

#[derive(Debug, Clone)]
pub struct ILSClearance {
    pub runway: String,
    pub threshold_coords: (f64, f64),
}

#[derive(Debug, Clone)]
pub enum GroundPoint {
    Coordinate(f64, f64),
    Stand(String),
    Push(String),
}

#[derive(Debug, Clone)]
pub struct ControllerData {
    pub controller: String,
    pub release_point: String,
}

impl Plane {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        callsign: String,
        squawk: u32,
        altitude: f64,
        heading: f64,
        speed: f64,
        lat: f64,
        lon: f64,
        vert_speed: f64,
        mode: PlaneMode,
        flight_plan: FlightPlan,
        currently_with_data: Option<ControllerData>,
        first_controller: Option<String>,
        stand: Option<String>,
    ) -> Self {
        let aircraft_type = flight_plan.aircraft_type.clone();
        let vref = Self::get_vref(&aircraft_type);
        
        Self {
            callsign,
            squawk,
            altitude,
            heading,
            speed,
            lat,
            lon,
            vert_speed,
            mode,
            flight_plan,
            currently_with_data,
            first_controller,
            stand,
            target_speed: speed,
            target_altitude: altitude,
            target_heading: heading,
            turn_dir: None,
            hold_fix: None,
            hold_start_time: None,
            aircraft_type,
            vref,
            old_alt: None,
            old_head: None,
            vert_mode: 0,
            cleared_ils: None,
            runway_heading: None,
            ground_position: None,
            ground_route: None,
            first_ground_position: None,
            lvl_coords: None,
            die_on_reaching_2k: false,
            last_time: Self::get_time(),
        }
    }

    fn get_time() -> f64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs_f64()
    }

    fn get_vref(aircraft_type: &str) -> f64 {
        // Simplified Vref lookup - could be expanded with actual data
        match aircraft_type {
            "B738" | "B737" => 140.0,
            "A320" | "A319" | "A321" => 138.0,
            "B77W" | "B777" => 145.0,
            "A333" | "A332" => 142.0,
            _ => 140.0,
        }
    }

    fn calculate_tas(&self) -> f64 {
        // True airspeed approximation: TAS = IAS * (1 + altitude/1000 * 0.02)
        self.speed * (1.0 + (self.altitude / 1000.0) * 0.02)
    }

    fn haversine_nm(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
        let r = 3440.065; // Earth radius in nautical miles
        let lat1_rad = lat1.to_radians();
        let lat2_rad = lat2.to_radians();
        let delta_lat = (lat2 - lat1).to_radians();
        let delta_lon = (lon2 - lon1).to_radians();

        let a = (delta_lat / 2.0).sin().powi(2)
            + lat1_rad.cos() * lat2_rad.cos() * (delta_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

        r * c
    }

    fn heading_from_to(from: (f64, f64), to: (f64, f64)) -> f64 {
        let lat1 = from.0.to_radians();
        let lat2 = to.0.to_radians();
        let delta_lon = (to.1 - from.1).to_radians();

        let y = delta_lon.sin() * lat2.cos();
        let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * delta_lon.cos();
        let theta = y.atan2(x);

        (theta.to_degrees() + 360.0) % 360.0
    }

    fn delta_lat_lon(&self, tas: f64, heading: f64, delta_t: f64) -> (f64, f64) {
        let distance_nm = tas * (delta_t / 3600.0);
        let lat_rad = self.lat.to_radians();
        let heading_rad = heading.to_radians();

        let delta_lat = (distance_nm / 60.0) * heading_rad.cos();
        let delta_lon = (distance_nm / 60.0) * heading_rad.sin() / lat_rad.cos();

        (delta_lat, delta_lon)
    }

    /// Generate FSD position update message
    pub fn position_update_text(&self) -> String {
        let display_heading = if self.stand.is_some() || self.mode == PlaneMode::GroundReady {
            (self.heading + 180.0) % 360.0
        } else {
            self.heading
        };

        let transponder_code = (100.0 / 9.0) * display_heading;

        format!(
            "@N:{}:{}:1:{}:{}:{}:{}:{}:0",
            self.callsign,
            self.squawk,
            self.lat,
            self.lon,
            self.altitude as i32,
            self.speed as i32,
            transponder_code as i32
        )
    }

    /// Main position calculation and update method
    pub fn calculate_position(&mut self) {
        let current_time = Self::get_time();
        let delta_t = current_time - self.last_time;
        self.last_time = current_time;

        if delta_t <= 0.0 {
            return;
        }

        // Calculate vertical mode and speed
        self.update_vertical_mode();
        self.update_speed(delta_t);
        self.update_altitude(delta_t);

        // Check if aircraft should be removed
        if self.die_on_reaching_2k && self.altitude <= 2000.0 {
            return;
        }

        // Mode-specific position updates
        match self.mode {
            PlaneMode::ILS => self.update_ils_mode(delta_t),
            PlaneMode::Heading => self.update_heading_mode(delta_t),
            PlaneMode::FlightPlan => self.update_flightplan_mode(delta_t),
            PlaneMode::GroundStationary => {},
            PlaneMode::GroundTaxi => self.update_ground_taxi(delta_t),
            PlaneMode::GroundReady => {},
            PlaneMode::None => {},
        }
    }

    fn update_vertical_mode(&mut self) {
        self.vert_mode = if self.altitude < self.target_altitude {
            1 // Climbing
        } else if self.altitude > self.target_altitude {
            -1 // Descending
        } else {
            0 // Level
        };

        // Set vertical speed based on mode
        if self.vert_mode == 1 {
            self.vert_speed = CLIMB_RATE;
        } else if self.vert_mode == -1 {
            self.vert_speed = DESCENT_RATE;
        }
    }

    fn update_speed(&mut self, delta_t: f64) {
        let speed_change_rate = 1.5; // knots per second
        
        if (self.target_speed - self.speed).abs() < speed_change_rate * delta_t {
            self.speed = self.target_speed;
        } else if self.target_speed > self.speed {
            self.speed += speed_change_rate * delta_t;
        } else if self.target_speed < self.speed {
            self.speed -= speed_change_rate * delta_t;
        }

        // Speed restrictions
        if self.altitude < 11000.0 && self.target_altitude < 10000.0 && self.target_speed > 250.0 {
            self.target_speed = 250.0;
        }

        if self.altitude > 10000.0 && self.altitude <= 10500.0 
            && self.target_altitude >= 10000.0 && self.target_speed <= 350.0 {
            self.target_speed = 350.0;
        }
    }

    fn update_altitude(&mut self, delta_t: f64) {
        if self.lvl_coords.is_some() && self.mode == PlaneMode::FlightPlan {
            // Level at specific coordinates
            if let Some(coords) = self.lvl_coords {
                let dist = Self::haversine_nm(self.lat, self.lon, coords.0, coords.1);
                if dist < 1.0 {
                    self.lvl_coords = None;
                    self.altitude = self.target_altitude;
                } else {
                    let delta_alt = self.target_altitude - self.altitude;
                    let tas = self.calculate_tas();
                    let approx_time = (dist / tas) * 60.0; // minutes
                    let approx_vert_speed = delta_alt / approx_time;
                    self.altitude += approx_vert_speed * (delta_t / 60.0);
                }
            }
        } else {
            // Normal altitude change
            self.altitude += self.vert_speed * (delta_t / 60.0);
        }

        // Constrain altitude to target
        if self.vert_speed > 0.0 && self.altitude >= self.target_altitude {
            self.altitude = self.target_altitude;
            self.vert_speed = 0.0;
        } else if self.vert_speed < 0.0 && self.altitude <= self.target_altitude {
            self.altitude = self.target_altitude;
            self.vert_speed = 0.0;
        }
    }

    fn update_ils_mode(&mut self, delta_t: f64) {
        if let Some(ref ils) = self.cleared_ils {
            let tas = self.calculate_tas();
            let (delta_lat, delta_lon) = self.delta_lat_lon(tas, self.heading, delta_t);
            
            let dist_out = Self::haversine_nm(
                self.lat, 
                self.lon, 
                ils.threshold_coords.0, 
                ils.threshold_coords.1
            );

            // 3-degree glideslope
            let required_altitude = (3.0_f64.to_radians().tan() * dist_out * 6076.0) + 0.0; // + airport elevation

            // Speed management on approach
            if dist_out < 4.0 {
                if self.speed > self.vref {
                    self.speed -= 0.75 * delta_t;
                }
                self.speed = self.speed.max(self.vref);
            }

            // Follow glideslope
            if self.altitude > required_altitude {
                if self.altitude - required_altitude > 1000.0 {
                    // Go around - too high
                    self.mode = PlaneMode::Heading;
                    self.cleared_ils = None;
                    self.target_altitude = 3000.0;
                    self.target_heading = self.heading;
                    self.target_speed = 220.0;
                    return;
                } else {
                    self.altitude = required_altitude;
                }
            }

            self.lat += delta_lat;
            self.lon += delta_lon;
        }
    }

    fn update_heading_mode(&mut self, delta_t: f64) {
        // Turn towards target heading
        if self.target_heading != self.heading {
            let angle_diff = (self.target_heading - self.heading + 360.0) % 360.0;
            
            if TURN_RATE * delta_t > angle_diff.min(360.0 - angle_diff) {
                self.heading = self.target_heading;
            } else if angle_diff < 180.0 {
                self.heading += TURN_RATE * delta_t;
            } else {
                self.heading -= TURN_RATE * delta_t;
            }

            self.heading = (self.heading + 360.0) % 360.0;
        }

        // Move forward
        let tas = self.calculate_tas();
        let (delta_lat, delta_lon) = self.delta_lat_lon(tas, self.heading, delta_t);
        self.lat += delta_lat;
        self.lon += delta_lon;
    }

    fn update_flightplan_mode(&mut self, delta_t: f64) {
        if self.flight_plan.route.fixes.is_empty() {
            self.mode = PlaneMode::Heading;
            return;
        }

        let tas = self.calculate_tas();
        let _distance_to_travel = tas * (delta_t / 3600.0);
        
        // Get next fix (simplified - would need fix database)
        let _next_fix = &self.flight_plan.route.fixes[0];
        // For now, we'll just use heading mode logic
        // In full implementation, would look up fix coordinates and navigate to them
        
        self.update_heading_mode(delta_t);
    }

    fn update_ground_taxi(&mut self, delta_t: f64) {
        if let Some(ref mut route) = self.ground_route {
            if route.is_empty() {
                self.mode = PlaneMode::GroundStationary;
                return;
            }

            let _distance_to_travel = TAXI_SPEED * (delta_t / 3600.0);
            // Simplified ground movement - full implementation would follow ground route
            
            self.mode = PlaneMode::GroundStationary;
        }
    }

    // Factory methods for creating planes in different scenarios
    
    /// Create a plane at a specific fix
    pub fn from_fix(
        callsign: String,
        _fix: String,
        squawk: u32,
        altitude: f64,
        heading: f64,
        speed: f64,
        vert_speed: f64,
        flight_plan: FlightPlan,
        currently_with_data: Option<ControllerData>,
        first_controller: Option<String>,
    ) -> Self {
        // For now, using placeholder coordinates
        let (lat, lon) = (51.15487, -0.16454);
        
        Self::new(
            callsign,
            squawk,
            altitude,
            heading,
            speed,
            lat,
            lon,
            vert_speed,
            PlaneMode::FlightPlan,
            flight_plan,
            currently_with_data,
            first_controller,
            None,
        )
    }

    /// Create a plane before a fix (20nm before)
    pub fn before_fix(
        callsign: String,
        _fix1: String,
        _fix2: String,
        squawk: u32,
        altitude: f64,
        heading: f64,
        speed: f64,
        vert_speed: f64,
        flight_plan: FlightPlan,
        currently_with_data: Option<ControllerData>,
        first_controller: Option<String>,
    ) -> Self {
        // In full implementation, would calculate position 20nm before fix1
        let (lat, lon) = (51.15487, -0.16454);
        
        Self::new(
            callsign,
            squawk,
            altitude,
            heading,
            speed,
            lat,
            lon,
            vert_speed,
            PlaneMode::FlightPlan,
            flight_plan,
            currently_with_data,
            first_controller,
            None,
        )
    }

    /// Create a plane at a ground point
    pub fn from_ground_point(
        callsign: String,
        _ground_point: String,
        squawk: u32,
        flight_plan: FlightPlan,
    ) -> Self {
        // In full implementation, would look up ground point coordinates
        let (lat, lon) = (51.15487, -0.16454);
        
        Self::new(
            callsign,
            squawk,
            0.0,
            0.0,
            0.0,
            lat,
            lon,
            0.0,
            PlaneMode::GroundStationary,
            flight_plan,
            None,
            None,
            None,
        )
    }

    /// Create a plane at a stand
    pub fn from_stand(
        callsign: String,
        stand: String,
        squawk: u32,
        flight_plan: FlightPlan,
    ) -> Self {
        // In full implementation, would look up stand coordinates
        let (lat, lon) = (51.15487, -0.16454);
        
        Self::new(
            callsign,
            squawk,
            0.0,
            0.0,
            0.0,
            lat,
            lon,
            0.0,
            PlaneMode::GroundStationary,
            flight_plan,
            None,
            None,
            Some(stand),
        )
    }

    /// Create a departing plane
    pub fn departure(
        callsign: String,
        _airport: String,
        squawk: u32,
        altitude: f64,
        heading: f64,
        speed: f64,
        vert_speed: f64,
        flight_plan: FlightPlan,
    ) -> Self {
        // In full implementation, would use runway coordinates
        let (lat, lon) = (51.15487, -0.16454);
        
        Self::new(
            callsign,
            squawk,
            altitude,
            heading,
            speed,
            lat,
            lon,
            vert_speed,
            PlaneMode::FlightPlan,
            flight_plan,
            None,
            None,
            None,
        )
    }

    // Command methods for ATC instructions
    
    pub fn set_heading(&mut self, heading: f64, turn_dir: Option<TurnDirection>) {
        self.target_heading = heading;
        self.turn_dir = turn_dir;
        if self.mode == PlaneMode::FlightPlan {
            self.mode = PlaneMode::Heading;
        }
    }

    pub fn set_altitude(&mut self, altitude: f64) {
        self.target_altitude = altitude;
    }

    pub fn set_speed(&mut self, speed: f64) {
        self.target_speed = speed;
    }

    pub fn clear_ils(&mut self, runway: String, threshold_coords: (f64, f64), runway_heading: f64) {
        self.cleared_ils = Some(ILSClearance {
            runway,
            threshold_coords,
        });
        self.runway_heading = Some(runway_heading);
        self.old_alt = Some(self.target_altitude);
        self.old_head = Some(self.target_heading);
    }

    pub fn resume_navigation(&mut self) {
        if self.mode == PlaneMode::Heading {
            self.mode = PlaneMode::FlightPlan;
        }
    }

    pub fn hold_at(&mut self, fix: String) {
        self.hold_fix = Some(fix);
    }

    pub fn cancel_hold(&mut self) {
        self.hold_fix = None;
        self.hold_start_time = None;
    }
}
