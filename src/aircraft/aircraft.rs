use crate::aircraft::flight_plan::FlightPlan;
use crate::utils::navigation::{
    haversine_nm, heading_from_to, position_bearing_distance, FixDatabase,
};

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
    pub runway_heading: f64,
    pub runway_elevation_ft: i32,
    pub cleared_heading: f64,
    pub intercept_started: bool,
}

/// Hold guidance definition
#[derive(Debug, Clone)]
pub struct HoldGuidance {
    pub fix: String,
    pub fix_lat: f64,
    pub fix_lon: f64,
    pub outbound_heading: i32,
    pub turn_right: bool,
    pub active: bool,
    pub leg_seconds: f64,
    pub leg_elapsed: f64,
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
    pub altitude: i32,     // feet
    pub heading: f64,      // degrees
    pub ground_speed: u32, // knots

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
    pub hold_guidance: Option<HoldGuidance>,

    // Departure info
    pub departure_runway: String,
    pub departure_heading: f64,

    // Target values
    pub target_altitude: i32,
    pub target_heading: f64,
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
        runway_heading: f64,
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

        tracing::info!(
            "[AIRCRAFT] Creating {} with {} route fixes: {:?}",
            callsign,
            route_fixes.len(),
            route_fixes
        );

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
            hold_guidance: None,
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

                        tracing::debug!(
                            "[AIRCRAFT] Found SID {} for runway {}: {} waypoints",
                            sid_name,
                            runway,
                            waypoints.len()
                        );
                        return waypoints;
                    }
                }
            }
            tracing::warn!(
                "[AIRCRAFT] SID {} not found for runway {} at {}",
                sid_name,
                runway,
                departure
            );
        } else {
            tracing::warn!("[AIRCRAFT] Could not read SID file: {}", sid_file);
        }

        Vec::new()
    }

    /// Parse route string to extract fix names
    fn parse_route(route: &str) -> Vec<String> {
        let mut fixes = Vec::new();

        // Split by spaces
        let parts: Vec<&str> = route
            .split(|c: char| c.is_whitespace())
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
                        chars.iter().take_while(|c| c.is_alphabetic()).count() <= 2 && has_digit;

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
    pub fn update(
        &mut self,
        delta_time: f64,
        fix_db: &FixDatabase,
        sim_config: &crate::config::SimulationConfig,
    ) {
        match self.phase {
            FlightPhase::OnGround => {
                // Wait a few seconds before starting takeoff
                if self.spawn_time.elapsed().as_secs() >= 5 {
                    self.phase = FlightPhase::Departing;
                    self.ground_speed = 10;
                    // tracing::info!("[{}] Starting takeoff roll", self.callsign);
                }
            }

            FlightPhase::Departing => {
                // Accelerate on runway
                if self.ground_speed < 150 {
                    self.ground_speed += (50.0 * delta_time) as u32;
                } else {
                    // tracing::info!(
                    //     "[{}] Rotation speed reached, route_fixes.len()={}",
                    //     self.callsign,
                    //     self.route_fixes.len()
                    // );
                    // Rotate and start climbing
                    self.phase = FlightPhase::Climbing;
                    self.altitude = 50;
                    self.target_speed = 250;

                    // Set initial heading towards first waypoint
                    if !self.route_fixes.is_empty() {
                        if let Some((fix_lat, fix_lon)) = fix_db.get(&self.route_fixes[0]) {
                            self.target_heading =
                                heading_from_to(self.latitude, self.longitude, *fix_lat, *fix_lon);
                            self.heading = self.target_heading;
                            // tracing::info!(
                            //     "[{}] Airborne, climbing to {} via {}",
                            //     self.callsign,
                            //     self.route_fixes[0],
                            //     self.route_fixes.join(" ")
                            // );
                        } else {
                            tracing::warn!(
                                "[{}] First waypoint {} not found in nav database",
                                self.callsign,
                                self.route_fixes[0]
                            );
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
                    if self.altitude >= self.target_altitude
                        && self.target_altitude < cruise_altitude
                    {
                        self.target_altitude = cruise_altitude;
                    }
                }

                // Above 10,000 ft, begin accelerating toward filed cruise speed.
                if self.lateral_mode != LateralMode::Ils && self.altitude > 10_000 {
                    if self.target_speed < self.flight_plan.cruise_speed {
                        self.target_speed = self.flight_plan.cruise_speed;
                    }
                }

                self.update_vertical_profile(delta_time, sim_config);
                self.update_speed_profile(delta_time);

                match self.lateral_mode {
                    LateralMode::FlightPlan => {
                        self.navigate_to_next_fix(fix_db, delta_time, sim_config)
                    }
                    LateralMode::Heading => {
                        if self.hold_guidance.as_ref().is_some_and(|hold| hold.active) {
                            self.update_hold_guidance(delta_time, sim_config);
                        } else {
                            self.turn_towards(
                                self.target_heading,
                                delta_time,
                                sim_config.turn_rate,
                            );
                        }
                    }
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

    fn update_vertical_profile(
        &mut self,
        delta_time: f64,
        sim_config: &crate::config::SimulationConfig,
    ) {
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
            self.ground_speed = self
                .ground_speed
                .saturating_sub(speed_step)
                .max(self.target_speed);
        }
    }

    // @ todo remove these to a utils place (I was lazy)

    fn normalize_heading(&self, mut heading: f64) -> f64 {
        while heading < 0.0 {
            heading += 360.0;
        }

        while heading >= 360.0 {
            heading -= 360.0;
        }

        heading
    }

    /// Initial bearing from point A to point B (degrees)
    fn initial_bearing(&self, lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
        heading_from_to(lat1, lon1, lat2, lon2)
    }

    // Returns shortest signed angle difference:
    // positive = turn right
    // negative = turn left
    fn shortest_angle_difference(&self, from: f64, to: f64) -> f64 {
        let mut diff = self.normalize_heading(to) - self.normalize_heading(from);

        while diff > 180.0 {
            diff -= 360.0;
        }

        while diff < -180.0 {
            diff += 360.0;
        }

        diff
    }

    fn heading_vector_xy_nm(heading: f64) -> (f64, f64) {
        let radians = heading.to_radians();
        (radians.sin(), radians.cos())
    }

    fn to_local_xy_nm(lat: f64, lon: f64, reference_lat: f64) -> (f64, f64) {
        let x = lon * 60.0 * reference_lat.to_radians().cos();
        let y = lat * 60.0;
        (x, y)
    }

    fn from_local_xy_nm(x: f64, y: f64, reference_lat: f64) -> (f64, f64) {
        let lat = y / 60.0;
        let cos_lat = reference_lat.to_radians().cos().max(1e-6);
        let lon = x / (60.0 * cos_lat);
        (lat, lon)
    }

    fn project_to_localizer_centerline(
        &self,
        runway_threshold: (f64, f64),
        runway_heading: f64,
    ) -> (f64, f64, f64) {
        let localizer_outbound = self.normalize_heading(runway_heading + 180.0);
        let reference_lat = (self.latitude + runway_threshold.0) / 2.0;
        let (ax, ay) = Self::to_local_xy_nm(self.latitude, self.longitude, reference_lat);
        let (lx, ly) = Self::to_local_xy_nm(runway_threshold.0, runway_threshold.1, reference_lat);
        let (dx, dy) = Self::heading_vector_xy_nm(localizer_outbound);

        let rx = ax - lx;
        let ry = ay - ly;
        let along_track_nm = rx * dx + ry * dy;
        let projected_x = lx + along_track_nm * dx;
        let projected_y = ly + along_track_nm * dy;
        let cross_track_nm = (rx * dy - ry * dx).abs();
        let (projected_lat, projected_lon) =
            Self::from_local_xy_nm(projected_x, projected_y, reference_lat);
        (projected_lat, projected_lon, cross_track_nm)
    }

    fn distance_to_localizer_intersection_nm(
        &self,
        runway_threshold: (f64, f64),
        runway_heading: f64,
    ) -> Option<f64> {
        let aircraft_lat = self.latitude;
        let aircraft_lon = self.longitude;
        let aircraft_heading = self.heading;
        let localizer_outbound = self.normalize_heading(runway_heading + 180.0);
        let reference_lat = (aircraft_lat + runway_threshold.0) / 2.0;

        let (ax, ay) = Self::to_local_xy_nm(aircraft_lat, aircraft_lon, reference_lat);
        let (lx, ly) = Self::to_local_xy_nm(runway_threshold.0, runway_threshold.1, reference_lat);
        let (adx, ady) = Self::heading_vector_xy_nm(aircraft_heading);
        let (ldx, ldy) = Self::heading_vector_xy_nm(localizer_outbound);

        let dx = lx - ax;
        let dy = ly - ay;

        // Solve:
        // aircraft + t * aircraft_dir = localizer_origin + u * localizer_dir
        // t is distance along current heading in NM (dir vectors are unit length in our local plane).
        let determinant = adx * (-ldy) - ady * (-ldx);
        if determinant.abs() < 1e-6 {
            return None;
        }

        let t = (dx * (-ldy) - dy * (-ldx)) / determinant;
        let u = (adx * dy - ady * dx) / determinant;

        if t >= 0.0 && u >= 0.0 {
            Some(t)
        } else {
            None
        }
    }

    fn update_ils_guidance(
        &mut self,
        delta_time: f64,
        sim_config: &crate::config::SimulationConfig,
    ) {
        let (
            threshold_lat,
            threshold_lon,
            runway_heading_raw,
            runway_elevation_ft,
            cleared_heading,
            intercept_started,
        ) = if let Some(ils) = self.ils_guidance.as_ref() {
            (
                ils.threshold_lat,
                ils.threshold_lon,
                ils.runway_heading,
                ils.runway_elevation_ft,
                ils.cleared_heading,
                ils.intercept_started,
            )
        } else {
            self.lateral_mode = LateralMode::FlightPlan;
            return;
        };

        let runway_heading = self.normalize_heading(runway_heading_raw);

        // ---------------------------------------------------------------------
        // DISTANCE / GLIDESLOPE
        // ---------------------------------------------------------------------

        let distance_nm = haversine_nm(self.latitude, self.longitude, threshold_lat, threshold_lon);
        let glideslope_altitude = ((distance_nm * 6076.0 * 3.0_f64.to_radians().tan()).round()
            as i32)
            + runway_elevation_ft;

        if self.altitude > glideslope_altitude + 100 {
            self.target_altitude = glideslope_altitude.max(runway_elevation_ft);
        }

        if distance_nm < 8.0 && self.target_speed > 160 {
            self.target_speed = 160;
        }
        if distance_nm < 4.0 {
            self.phase = FlightPhase::Approach;
        }
        if distance_nm < 1.0 {
            self.target_altitude = runway_elevation_ft;
            self.phase = FlightPhase::Landing;
        }

        // ---------------------------------------------------------------------
        // LOCALIZER INTERCEPT / TRACK
        // ---------------------------------------------------------------------

        // Signed localizer error where (for this bearing convention):
        // negative => aircraft is right of centerline
        // positive => aircraft is left of centerline
        let localizer_outbound = self.normalize_heading(runway_heading + 180.0);
        let bearing_from_threshold =
            self.initial_bearing(threshold_lat, threshold_lon, self.latitude, self.longitude);
        let localizer_error =
            self.shortest_angle_difference(localizer_outbound, bearing_from_threshold);
        let (snap_lat, snap_lon, cross_track_nm) =
            self.project_to_localizer_centerline((threshold_lat, threshold_lon), runway_heading);

        const CENTERLINE_SNAP_TOLERANCE_NM: f64 = 0.03;
        if cross_track_nm <= CENTERLINE_SNAP_TOLERANCE_NM {
            self.latitude = snap_lat;
            self.longitude = snap_lon;
            self.heading = runway_heading;
            self.target_heading = runway_heading;
            if let Some(ils) = self.ils_guidance.as_mut() {
                ils.intercept_started = true;
            }

            tracing::info!(
                "[ILS DEBUG {}] centerline-snap rwy {:.1} cross_track {:.3}nm tol {:.3} heading {:.1}",
                self.callsign,
                runway_heading,
                cross_track_nm,
                CENTERLINE_SNAP_TOLERANCE_NM,
                runway_heading
            );
            return;
        }

        let angle_to_runway_course = self
            .shortest_angle_difference(self.heading, runway_heading)
            .abs();
        let turn_rate = sim_config.turn_rate.max(1.0);
        let turn_time_seconds = angle_to_runway_course / turn_rate;
        let effective_speed = self.ground_speed.max(self.target_speed).max(120) as f64;
        let turn_distance_nm = effective_speed * turn_time_seconds / 3600.0;
        let step_distance_nm = effective_speed * delta_time / 3600.0;
        let distance_to_intersection_nm = self
            .distance_to_localizer_intersection_nm((threshold_lat, threshold_lon), runway_heading);

        // Hold the assigned heading until the geometric "last-second" turn point.
        let should_start_intercept =
            distance_to_intersection_nm.is_some_and(|distance| distance <= turn_distance_nm);

        if should_start_intercept && !intercept_started {
            if let Some(ils) = self.ils_guidance.as_mut() {
                ils.intercept_started = true;
            }
        }

        let intercept_active = intercept_started || should_start_intercept;

        if !intercept_active {
            self.target_heading = cleared_heading;
            tracing::info!(
                "[ILS DEBUG {}] hold-vector rwy {:.1} cur {:.1} tgt {:.1} clr {:.1} dist {:.2}nm loc_err {:.2} angle_to_course {:.2} intx_dist {:?} turn_dist {:.2} step {:.3}",
                self.callsign,
                runway_heading,
                self.heading,
                self.target_heading,
                cleared_heading,
                distance_nm,
                localizer_error,
                angle_to_runway_course,
                distance_to_intersection_nm,
                turn_distance_nm,
                step_distance_nm
            );
            self.turn_towards(self.target_heading, delta_time, sim_config.turn_rate);
            return;
        }

        const FINAL_TURN_SNAP_TOLERANCE_NM: f64 = 0.08;
        if angle_to_runway_course <= 1.0 && cross_track_nm <= FINAL_TURN_SNAP_TOLERANCE_NM {
            self.latitude = snap_lat;
            self.longitude = snap_lon;
            self.heading = runway_heading;
            self.target_heading = runway_heading;
            tracing::info!(
                "[ILS DEBUG {}] post-turn-snap rwy {:.1} cross_track {:.3}nm tol {:.3} angle_to_course {:.2}",
                self.callsign,
                runway_heading,
                cross_track_nm,
                FINAL_TURN_SNAP_TOLERANCE_NM,
                angle_to_runway_course
            );
            return;
        }

        let previous_target_heading = self.target_heading;
        let (desired_heading, guidance_mode, guidance_value) = if localizer_error.abs() <= 1.0 {
            // Once captured, apply small proportional corrections to stay on localizer.
            let correction = (localizer_error * 1.5).clamp(-8.0, 8.0);
            (
                self.normalize_heading(runway_heading + correction),
                "track",
                correction,
            )
        } else {
            // Continuous turn: once triggered, go straight to final course.
            (
                runway_heading,
                "continuous-turn-final",
                self.shortest_angle_difference(self.heading, runway_heading),
            )
        };

        tracing::info!(
            "[ILS DEBUG {}] {} rwy {:.1} cur {:.1} prev_tgt {:.1} desired {:.1} clr {:.1} active {} trigger_now {} dist {:.2}nm loc_err {:.2} angle_to_course {:.2} intx_dist {:?} turn_dist {:.2} step {:.3} guidance {:.2}",
            self.callsign,
            guidance_mode,
            runway_heading,
            self.heading,
            previous_target_heading,
            desired_heading,
            cleared_heading,
            intercept_active,
            should_start_intercept,
            distance_nm,
            localizer_error,
            angle_to_runway_course,
            distance_to_intersection_nm,
            turn_distance_nm,
            step_distance_nm,
            guidance_value
        );
        self.target_heading = desired_heading;
        self.turn_towards(desired_heading, delta_time, sim_config.turn_rate);
    }

    fn update_hold_guidance(
        &mut self,
        delta_time: f64,
        sim_config: &crate::config::SimulationConfig,
    ) {
        let Some(hold_state) = self.hold_guidance.as_ref() else {
            self.lateral_mode = LateralMode::FlightPlan;
            return;
        };

        let turn_right = hold_state.turn_right;
        let fix_lat = hold_state.fix_lat;
        let fix_lon = hold_state.fix_lon;
        let outbound_heading = hold_state.outbound_heading;

        if !hold_state.active {
            return;
        }

        if self.heading != self.target_heading {
            self.turn_towards_with_direction(
                self.target_heading,
                delta_time,
                sim_config.turn_rate,
                Some(turn_right),
            );

            if self.heading == self.target_heading {
                if let Some(hold) = self.hold_guidance.as_mut() {
                    hold.leg_elapsed = 0.0;
                }
            }

            return;
        }

        let mut elapsed_ready = false;
        if let Some(hold) = self.hold_guidance.as_mut() {
            hold.leg_elapsed += delta_time;
            if hold.leg_elapsed >= hold.leg_seconds {
                hold.leg_elapsed = 0.0;
                elapsed_ready = true;
            }
        }

        if !elapsed_ready {
            return;
        }

        // When close to the hold fix, snap over it and restart with the
        // published outbound heading before reversing back inbound.
        if haversine_nm(self.latitude, self.longitude, fix_lat, fix_lon) < 0.5 {
            self.latitude = fix_lat;
            self.longitude = fix_lon;
            self.heading = outbound_heading as f64;
            self.target_heading = outbound_heading as f64;
        }

        self.target_heading = (self.target_heading + 180.0) % 360.0;
    }

    /// Navigate towards the next fix
    fn navigate_to_next_fix(
        &mut self,
        fix_db: &FixDatabase,
        delta_time: f64,
        sim_config: &crate::config::SimulationConfig,
    ) {
        while self.current_fix_index < self.route_fixes.len() {
            let current_fix = self.route_fixes[self.current_fix_index].clone();

            let Some((fix_lat, fix_lon)) = fix_db.get(&current_fix) else {
                tracing::warn!(
                    "[{}] Fix {} missing from nav db, skipping",
                    self.callsign,
                    current_fix
                );
                self.current_fix_index += 1;
                continue;
            };

            // Calculate distance to fix
            let distance = haversine_nm(self.latitude, self.longitude, *fix_lat, *fix_lon);

            // Calculate required heading to fix
            let required_heading =
                heading_from_to(self.latitude, self.longitude, *fix_lat, *fix_lon);

            // If within 0.5 NM of fix, move to next fix and evaluate again this tick.
            if distance < 0.5 {
                if self.should_activate_hold(&current_fix) {
                    self.activate_hold_mode();
                    return;
                }

                self.current_fix_index += 1;

                if self.current_fix_index < self.route_fixes.len() {
                    let next_fix = &self.route_fixes[self.current_fix_index];
                    // tracing::info!(
                    //     "[{}] Passed {}, turning to next waypoint: {}",
                    //     self.callsign,
                    //     current_fix,
                    //     next_fix
                    // );
                }
                continue;
            }

            // Always turn towards the current fix (whether we just updated it or not)
            self.target_heading = required_heading;
            self.turn_towards(required_heading, delta_time, sim_config.turn_rate);
            return;
        }
    }

    /// Turn towards a target heading
    fn turn_towards(&mut self, target: f64, delta_time: f64, turn_rate: f64) {
        self.turn_towards_with_direction(target, delta_time, turn_rate, None);
    }

    fn turn_towards_with_direction(
        &mut self,
        target: f64,
        delta_time: f64,
        turn_rate: f64,
        preferred_right: Option<bool>,
    ) {
        let diff: f64 = ((target - self.heading + 540.0) % 360.0) - 180.0;

        if diff.abs() < 2.0 {
            self.heading = target;
        } else {
            // Calculate turn amount as float first, then convert to int (fixes rounding to 0)
            let turn_amount_f = turn_rate * delta_time;
            let turn_amount = turn_amount_f.max(1.0); // Ensure at least 1 degree per update

            match preferred_right {
                Some(true) => {
                    let right_diff = (target - self.heading + 360.0) % 360.0;
                    self.heading += turn_amount.min(right_diff);
                }
                Some(false) => {
                    let left_diff = (self.heading - target + 360.0) % 360.0;
                    self.heading -= turn_amount.min(left_diff);
                }
                None => {
                    if diff > 0.0 {
                        self.heading += turn_amount.min(diff);
                    } else {
                        self.heading -= turn_amount.min(diff.abs());
                    }
                }
            }

            // Normalize heading
            self.heading = (self.heading + 360.0) % 360.0;
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
            distance_nm,
        );

        self.latitude = new_lat;
        self.longitude = new_lon;
    }

    /// Mark which controller currently assumes this aircraft.
    pub fn set_assumed_by(&mut self, controller: Option<String>) {
        self.assumed_by = controller;
    }

    /// Apply a heading assignment and switch to heading mode.
    pub fn assign_heading(&mut self, heading: f64) {
        self.target_heading = (heading % 360.0 + 360.0) % 360.0;
        self.lateral_mode = LateralMode::Heading;
        self.ils_guidance = None;
        self.hold_guidance = None;
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
        runway_heading: f64,
        runway_elevation_ft: i32,
    ) {
        self.ils_guidance = Some(IlsGuidance {
            runway,
            threshold_lat: threshold.0,
            threshold_lon: threshold.1,
            runway_heading,
            runway_elevation_ft,
            cleared_heading: self.target_heading,
            intercept_started: false,
        });

        self.auto_climb_to_cruise = false;
        self.lateral_mode = LateralMode::Ils;
        self.phase = FlightPhase::Approach;
        self.hold_guidance = None;

        if self.target_speed > 200 {
            self.target_speed = 200;
        }
    }

    /// Return to route-based navigation mode.
    pub fn resume_navigation(&mut self) {
        self.lateral_mode = LateralMode::FlightPlan;
        self.ils_guidance = None;
        self.hold_guidance = None;
    }

    /// Append fixes to the existing route, skipping immediate duplicates.
    pub fn append_route_fixes(&mut self, fixes: Vec<String>) -> usize {
        let mut added = 0;

        for fix in fixes {
            let fix = fix.to_uppercase();
            if self.route_fixes.last().is_some_and(|last| last == &fix) {
                continue;
            }

            self.route_fixes.push(fix);
            added += 1;
        }

        added
    }

    /// Arm a hold at a fix. Hold activates when crossing that fix while route-following.
    pub fn assign_hold(&mut self, fix: &str, fix_db: &FixDatabase) -> bool {
        let hold_fix = fix.to_uppercase();
        let Some((fix_lat, fix_lon)) = fix_db.get(&hold_fix) else {
            return false;
        };

        let (outbound_heading, turn_right) = Self::hold_defaults(&hold_fix);

        self.hold_guidance = Some(HoldGuidance {
            fix: hold_fix,
            fix_lat: *fix_lat,
            fix_lon: *fix_lon,
            outbound_heading,
            turn_right,
            active: false,
            leg_seconds: 55.0,
            leg_elapsed: 0.0,
        });

        true
    }

    /// Proceed direct to a fix. If it is not in the remaining route but exists in
    /// nav data, insert it as the immediate next waypoint.
    pub fn direct_to_fix(&mut self, fix: &str, fix_db: &FixDatabase) -> bool {
        let target_fix = fix.to_uppercase();

        if self.current_fix_index >= self.route_fixes.len() {
            if fix_db.contains_key(&target_fix) {
                self.route_fixes.push(target_fix.clone());
                if !self.route_fixes.is_empty() {
                    self.current_fix_index = self.route_fixes.len() - 1;
                }
                self.lateral_mode = LateralMode::FlightPlan;
                self.ils_guidance = None;
                self.hold_guidance = None;

                if let Some((fix_lat, fix_lon)) = fix_db.get(&target_fix) {
                    self.target_heading =
                        heading_from_to(self.latitude, self.longitude, *fix_lat, *fix_lon);
                }

                return true;
            }

            return false;
        }
        let search_slice = &self.route_fixes[self.current_fix_index..];

        if let Some(offset) = search_slice.iter().position(|f| f == &target_fix) {
            self.current_fix_index += offset;
            self.lateral_mode = LateralMode::FlightPlan;
            self.ils_guidance = None;
            self.hold_guidance = None;

            if let Some((fix_lat, fix_lon)) = fix_db.get(&target_fix) {
                self.target_heading =
                    heading_from_to(self.latitude, self.longitude, *fix_lat, *fix_lon);
            }

            true
        } else if fix_db.contains_key(&target_fix) {
            self.route_fixes.insert(self.current_fix_index, target_fix);
            self.lateral_mode = LateralMode::FlightPlan;
            self.ils_guidance = None;
            self.hold_guidance = None;

            let inserted_fix = &self.route_fixes[self.current_fix_index];
            if let Some((fix_lat, fix_lon)) = fix_db.get(inserted_fix) {
                self.target_heading =
                    heading_from_to(self.latitude, self.longitude, *fix_lat, *fix_lon);
            }

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
        self.route_fixes
            .get(self.current_fix_index)
            .map(|s| s.as_str())
    }

    /// Check if aircraft has completed its route
    pub fn is_route_complete(&self) -> bool {
        self.current_fix_index >= self.route_fixes.len()
    }

    fn should_activate_hold(&self, current_fix: &str) -> bool {
        self.hold_guidance
            .as_ref()
            .is_some_and(|hold| !hold.active && current_fix.eq_ignore_ascii_case(&hold.fix))
    }

    fn activate_hold_mode(&mut self) {
        let Some(hold) = self.hold_guidance.as_mut() else {
            return;
        };

        self.lateral_mode = LateralMode::Heading;
        self.heading = hold.outbound_heading as f64;
        self.target_heading = hold.outbound_heading as f64;
        hold.active = true;
        hold.leg_elapsed = hold.leg_seconds + 1.0;

        tracing::info!(
            "[{}] Entering hold at {} outbound {} {} turns",
            self.callsign,
            hold.fix,
            hold.outbound_heading,
            if hold.turn_right { "right" } else { "left" }
        );
    }

    fn hold_defaults(fix: &str) -> (i32, bool) {
        match fix {
            "BIG" => (302, true),
            "LAM" => (262, false),
            "BNN" => (116, true),
            "OCK" => (328, true),
            "TIMBA" => (307, true),
            "WILLO" => (284, false),
            "JACKO" => (264, false),
            "GODLU" => (309, true),
            "DAYNE" => (311, true),
            "ROSUN" => (172, true),
            "MIRSI" => (61, true),
            "TARTN" => (15, false),
            "STIRA" => (233, true),
            _ => (307, true),
        }
    }
}
