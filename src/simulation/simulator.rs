use anyhow::Result;
use std::sync::Arc;
use std::collections::HashMap;
use tracing::{info, debug, warn};
use tokio::time::{interval, Duration};
use tokio::sync::mpsc::{UnboundedReceiver, error::TryRecvError};
use rand::Rng;

use crate::scenario::Scenario;
use crate::config::{SimulationConfig, FleetConfig};
use crate::utils::navigation::{FixDatabase, sf_coords_to_decimal};
use crate::utils::performance::PerformanceDatabase;
use crate::aircraft::Aircraft;
use super::ai_controller::AiController;
use super::ai_pilot::AiPilot;

/// Main simulation controller
pub struct Simulator {
    scenario: Arc<Scenario>,
    sim_config: Arc<SimulationConfig>,
    fleet_config: Arc<FleetConfig>,
    nav_db: Arc<FixDatabase>,
    perf_db: Arc<PerformanceDatabase>,
    server_addr: String,
    ai_controllers: Vec<AiController>,
    aircraft: Vec<Aircraft>,
    pilot_clients: HashMap<String, AiPilot>,
    controller_message_rx: Option<UnboundedReceiver<String>>,
    running: bool,
    squawk_pool: Vec<u16>,
    used_callsigns: std::collections::HashSet<String>,
}

impl Simulator {
    /// Create a new simulator
    pub fn new(
        scenario: Scenario,
        sim_config: SimulationConfig,
        fleet_config: FleetConfig,
        nav_db: Arc<FixDatabase>,
        perf_db: Arc<PerformanceDatabase>,
        server_addr: String,
    ) -> Self {
        Self {
            scenario: Arc::new(scenario),
            sim_config: Arc::new(sim_config),
            fleet_config: Arc::new(fleet_config),
            nav_db,
            perf_db,
            server_addr,
            ai_controllers: Vec::new(),
            aircraft: Vec::new(),
            pilot_clients: HashMap::new(),
            controller_message_rx: None,
            running: false,
            squawk_pool: crate::config::get_ccams_squawks(),
            used_callsigns: std::collections::HashSet::new(),
        }
    }

    /// Initialize the simulation
    pub async fn initialize(&mut self) -> Result<()> {
        info!("[SIMULATOR] Initializing simulation...");
        
        // Display scenario information
        let stats = self.scenario.statistics();
        info!("{}", stats);
        
        // Login AI controllers
        self.login_ai_controllers().await?;
        
        info!("[SIMULATOR] Initialization complete");
        Ok(())
    }

    /// Login AI controllers to the FSD server
    async fn login_ai_controllers(&mut self) -> Result<()> {
        info!("[SIMULATOR] Logging in AI controllers...");
        
        let (master_callsign, master_freq) = self.scenario.master_controller();
        
        // Create and login master controller
        info!("[SIMULATOR] Creating master controller: {} on {}", master_callsign, master_freq);
        
        let mut master_controller = AiController::new(
            master_callsign.to_string(),
            master_freq.to_string(),
            51.5,  // Default latitude (central UK)
            -0.5,  // Default longitude
            300,   // Range in nautical miles
        );
        
        // Connect and login
        master_controller.connect(&self.server_addr).await?;
        master_controller.login().await?;
        
        // Wait a bit for the server to process
        tokio::time::sleep(Duration::from_millis(500)).await;
        
        // Send IP query
        master_controller.send_ip_query().await?;
        
        // Start message loop
        self.controller_message_rx = master_controller.start_message_loop(true).await?;
        
        self.ai_controllers.push(master_controller);
        
        info!("[SIMULATOR] Master controller {} logged in", master_callsign);
        
        // Login other controllers
        for (callsign, freq) in self.scenario.other_controllers() {
            info!("[SIMULATOR] Creating controller: {} on {}", callsign, freq);
            
            let mut controller = AiController::new(
                callsign.clone(),
                freq.clone(),
                51.5,
                -0.5,
                300,
            );
            
            controller.connect(&self.server_addr).await?;
            
            // Wait a bit between logins
            tokio::time::sleep(Duration::from_millis(200)).await;
            
            controller.login().await?;
            tokio::time::sleep(Duration::from_millis(300)).await;
            
            controller.send_ip_query().await?;
            let _ = controller.start_message_loop(false).await?;
            
            self.ai_controllers.push(controller);
            
            info!("[SIMULATOR] Controller {} logged in", callsign);
        }
        
        info!("[SIMULATOR] {} AI controllers logged in", self.ai_controllers.len());
        
        Ok(())
    }

    /// Start the main simulation loop
    pub async fn run(&mut self, shutdown: tokio::sync::broadcast::Receiver<()>) -> Result<()> {
        info!("[SIMULATOR] Starting main simulation loop...");
        self.running = true;
        
        // Create timers for different spawn intervals
        let mut departure_timers = self.create_departure_timers();
        let mut transit_timers = self.create_transit_timers();
        
        // Main update loop (runs at radar update rate)
        let radar_update_ms = (1000.0 / self.sim_config.radar_update_rate) as u64;
        let mut update_interval = interval(Duration::from_millis(radar_update_ms));
        
        let mut loop_count = 0u64;
        let mut shutdown_rx = shutdown;
        
        loop {
            tokio::select! {
                _ = shutdown_rx.recv() => {
                    info!("[SIMULATOR] Shutdown signal received");
                    break;
                }
                _ = update_interval.tick() => {
                    loop_count += 1;
                    
                    let delta_time = (radar_update_ms as f64) / 1000.0;
                    
                    // Check departure timers
                    self.check_departure_spawns(&mut departure_timers, loop_count).await?;
                    
                    // Check transit timers
                    self.check_transit_spawns(&mut transit_timers, loop_count).await?;

                    // Apply controller-issued commands to simulated aircraft
                    self.process_controller_messages().await?;
                    
                    // Update all aircraft
                    self.update_aircraft(delta_time);
                    
                    // Send pilot position updates every 5 seconds (25 ticks at 5 Hz)
                    if loop_count % 25 == 0 {
                        self.broadcast_pilot_positions().await?;
                    }
                    
                    // Log status periodically
                    if loop_count % 50 == 0 {
                        debug!("[SIMULATOR] Loop {}: {} controllers, {} aircraft", 
                               loop_count, self.ai_controllers.len(), self.aircraft.len());
                    }
                }
            }
        }
        
        self.running = false;
        info!("[SIMULATOR] Simulation loop stopped");
        Ok(())
    }

    async fn process_controller_messages(&mut self) -> Result<()> {
        let mut buffered_messages = Vec::new();
        let mut disconnected = false;

        if let Some(rx) = self.controller_message_rx.as_mut() {
            loop {
                match rx.try_recv() {
                    Ok(message) => buffered_messages.push(message),
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        disconnected = true;
                        break;
                    }
                }
            }
        }

        if disconnected {
            warn!("[SIMULATOR] Master controller message channel disconnected");
            self.controller_message_rx = None;
        }

        for message in buffered_messages {
            if message.starts_with("$CQ") && message.contains("@94835") {
                info!("[SIMULATOR CTRL RX] {}", message);
            } else if message.starts_with("$HA") {
                info!("[SIMULATOR HANDOFF RX] {}", message);
            }
            self.handle_controller_message(&message).await?;
        }

        Ok(())
    }

    async fn handle_controller_message(&mut self, message: &str) -> Result<()> {
        // Handle handoff accept messages that clear ownership.
        if message.starts_with("$HA") {
            let parts: Vec<&str> = message.split(':').collect();
            if parts.len() >= 3 {
                let callsign = parts[2];
                if let Some(aircraft) = self.aircraft.iter_mut().find(|a| a.callsign == callsign) {
                    aircraft.set_assumed_by(None);
                }
            }
            return Ok(());
        }

        if !message.starts_with("$CQ") {
            return Ok(());
        }

        let parts: Vec<&str> = message.split(':').collect();
        if parts.len() < 4 {
            return Ok(());
        }

        let from_controller = parts[0].trim_start_matches("$CQ");
        let target = parts[1];
        let command = parts[2];

        // Sim command channel used by legacy sweatbox control messages.
        if target != "@94835" {
            if command == "PD" || command == "ROND" || command == "SC" || command == "TA" || command == "IT" || command == "BC" || command == "DR" {
                info!("[SIMULATOR CTRL DROP] target {} for {}", target, message);
            }
            return Ok(());
        }

        let mut remove_callsign: Option<String> = None;

        match command {
            "IT" => {
                let callsign = parts[3];
                if let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) {
                    if let Some(owner) = self.aircraft[index].assumed_by.as_deref() {
                        if !owner.eq_ignore_ascii_case(from_controller.trim()) {
                            warn!("[SIMULATOR] {} cannot assume {}; currently assumed by {}", from_controller, callsign, owner);
                            return Ok(());
                        }
                    }

                    self.aircraft[index].set_assumed_by(Some(from_controller.trim().to_string()));
                    info!("[SIMULATOR] {} assumed by {}", callsign, from_controller);
                }
            }
            "SC" => {
                if parts.len() < 5 {
                    return Ok(());
                }

                let callsign = parts[3];
                let instruction = parts[4];

                if let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) {
                    if !Self::controller_has_assumed_tag(&self.aircraft[index], from_controller) {
                        warn!("[SIMULATOR] {} ignored SC for {} (tag not assumed)", from_controller, callsign);
                        return Ok(());
                    }

                    if let Some(value) = instruction.strip_prefix('H') {
                        if let Ok(target_heading) = value.parse::<i32>() {
                            self.aircraft[index].assign_heading(target_heading);
                            info!("[SIMULATOR] {} heading {}", callsign, target_heading);
                        }
                    } else if let Some(value) = instruction.strip_prefix('S') {
                        if let Ok(target_speed) = value.parse::<u32>() {
                            self.aircraft[index].assign_speed(target_speed);
                            info!("[SIMULATOR] {} speed {}", callsign, target_speed);
                        }
                    } else if let Some(value) = instruction.strip_prefix('M') {
                        if let Ok(mach_times_100) = value.parse::<u32>() {
                            let ias = (((mach_times_100 as f64) / 100.0) * (450.0 / 0.7842)).round() as u32;
                            self.aircraft[index].assign_speed(ias);
                            info!("[SIMULATOR] {} speed M{} (~{}kt)", callsign, mach_times_100, ias);
                        }
                    } else {
                        // Some clients encode direct/route payloads via SC:<callsign>:<FIX>
                        self.apply_route_payload(callsign, instruction, from_controller)?;
                    }
                }
            }
            "TA" => {
                if parts.len() < 5 {
                    return Ok(());
                }

                let callsign = parts[3];
                if let Ok(mut target_altitude) = parts[4].parse::<i32>() {
                    if target_altitude == 1 {
                        self.apply_route_payload(callsign, "ILS", from_controller)?;
                        return Ok(());
                    }

                    if let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) {
                        if !Self::controller_has_assumed_tag(&self.aircraft[index], from_controller) {
                            warn!("[SIMULATOR] {} ignored TA for {} (tag not assumed)", from_controller, callsign);
                            return Ok(());
                        }

                        if target_altitude == 0 {
                            target_altitude = self.aircraft[index].flight_plan.cruise_altitude as i32 * 100;
                        }

                        let aircraft = &mut self.aircraft[index];
                        aircraft.assign_altitude(target_altitude);
                        info!("[SIMULATOR] {} altitude {}", callsign, target_altitude);
                    }
                }
            }
            "BC" => {
                if parts.len() < 5 {
                    return Ok(());
                }

                let callsign = parts[3];
                let squawk = parts[4];

                if squawk == "7000" {
                    return Ok(());
                }

                if let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) {
                    if !Self::controller_has_assumed_tag(&self.aircraft[index], from_controller) {
                        warn!("[SIMULATOR] {} ignored BC for {} (tag not assumed)", from_controller, callsign);
                        return Ok(());
                    }

                    let aircraft = &mut self.aircraft[index];
                    aircraft.squawk = squawk.to_string();
                    info!("[SIMULATOR] {} squawk {}", callsign, squawk);
                }
            }
            "DR" => {
                let callsign = parts[3];
                if let Some(aircraft) = self.aircraft.iter().find(|a| a.callsign == callsign) {
                    if !Self::controller_has_assumed_tag(aircraft, from_controller) {
                        warn!("[SIMULATOR] {} ignored DR for {} (tag not assumed)", from_controller, callsign);
                        return Ok(());
                    }
                    remove_callsign = Some(callsign.to_string());
                }
            }
            "PD" | "ROND" => {
                if parts.len() >= 5 {
                    let callsign = parts[3];
                    let payload = Self::payload_from_parts(&parts, 4);
                    self.apply_route_payload(callsign, &payload, from_controller)?;
                }
            }
            _ => {
                // Handle direct-to style packets, e.g. ...:<callsign>:DVR
                if parts.len() >= 5 {
                    let callsign = parts[3];
                    let payload = Self::payload_from_parts(&parts, 4);
                    self.apply_route_payload(callsign, &payload, from_controller)?;
                }
            }
        }

        if let Some(callsign) = remove_callsign {
            self.remove_aircraft_by_callsign(&callsign).await?;
        }

        Ok(())
    }

    async fn remove_aircraft_by_callsign(&mut self, callsign: &str) -> Result<()> {
        let removed_squawk = self.aircraft
            .iter()
            .find(|a| a.callsign == callsign)
            .and_then(|a| a.squawk.parse::<u16>().ok());

        let before = self.aircraft.len();
        self.aircraft.retain(|a| a.callsign != callsign);

        if before == self.aircraft.len() {
            return Ok(());
        }

        self.used_callsigns.remove(callsign);
        if let Some(squawk) = removed_squawk {
            self.squawk_pool.push(squawk);
        }

        if let Some(mut pilot) = self.pilot_clients.remove(callsign) {
            pilot.disconnect().await?;
        }

        info!("[SIMULATOR] Removed aircraft {}", callsign);
        Ok(())
    }

    fn controller_has_assumed_tag(aircraft: &Aircraft, controller: &str) -> bool {
        aircraft
            .assumed_by
            .as_deref()
            .map(str::trim)
            .is_some_and(|owner| owner.eq_ignore_ascii_case(controller.trim()))
    }

    fn payload_from_parts(parts: &[&str], start_index: usize) -> String {
        if start_index >= parts.len() {
            String::new()
        } else {
            parts[start_index..]
                .iter()
                .map(|segment| segment.trim())
                .filter(|segment| !segment.is_empty())
                .collect::<Vec<_>>()
                .join(" ")
        }
    }

    fn apply_route_payload(&mut self, callsign: &str, payload: &str, from_controller: &str) -> Result<()> {
        let payload = payload.trim();
        if payload.is_empty() {
            return Ok(());
        }

        if payload.eq_ignore_ascii_case("HOLD") {
            info!("[SIMULATOR] {} HOLD not implemented", callsign);
            return Ok(());
        }

        if payload.eq_ignore_ascii_case("ILS") {
            return self.assign_ils(callsign, from_controller);
        }

        let normalized_payload = payload.strip_prefix("LVL").unwrap_or(payload).trim();

        let direct_fix = normalized_payload
            .split(|c: char| !c.is_ascii_alphabetic())
            .filter(|token| token.len() >= 2)
            .map(|token| token.to_uppercase())
            .find(|token| {
                token != "DCT"
                    && token != "DIRECT"
                    && token != "NAV"
                    && token != "OWN"
                    && self.nav_db.contains_key(token)
            })
            .unwrap_or_default();

        if direct_fix.is_empty() {
            warn!("[SIMULATOR] {} ignored direct payload '{}' for {} (no usable fix)", from_controller, payload, callsign);
            return Ok(());
        }

        if let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) {
            if !Self::controller_has_assumed_tag(&self.aircraft[index], from_controller) {
                warn!("[SIMULATOR] {} ignored direct for {} (tag not assumed)", from_controller, callsign);
                return Ok(());
            }

            let aircraft = &mut self.aircraft[index];
            if aircraft.direct_to_fix(&direct_fix, &self.nav_db) {
                info!("[SIMULATOR] {} direct {}", callsign, direct_fix);
            } else {
                warn!("[SIMULATOR] {} ignored direct {} for {} (fix unavailable)", from_controller, direct_fix, callsign);
            }
        }

        Ok(())
    }

    fn assign_ils(&mut self, callsign: &str, from_controller: &str) -> Result<()> {
        let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) else {
            return Ok(());
        };

        if !Self::controller_has_assumed_tag(&self.aircraft[index], from_controller) {
            warn!("[SIMULATOR] {} ignored ILS for {} (tag not assumed)", from_controller, callsign);
            return Ok(());
        }

        let destination = self.aircraft[index].flight_plan.arrival.clone();
        let Some(active_runway) = self.scenario.active_runway(&destination).map(|r| r.to_string()) else {
            warn!("[SIMULATOR] No active runway for destination {} ({} ILS ignored)", destination, callsign);
            return Ok(());
        };

        let Some((runway_heading, threshold_lat, threshold_lon)) =
            self.lookup_runway_endpoint(&destination, &active_runway)? else {
            warn!("[SIMULATOR] Could not resolve runway {} for {} ({} ILS ignored)", active_runway, destination, callsign);
            return Ok(());
        };

        let runway_elevation_ft = self.sim_config
            .airport_elevations
            .get(&destination)
            .copied()
            .unwrap_or(0) as i32;

        let aircraft = &mut self.aircraft[index];
        aircraft.assign_ils(
            active_runway.clone(),
            (threshold_lat, threshold_lon),
            runway_heading,
            runway_elevation_ft,
        );

        info!("[SIMULATOR] {} ILS {} {}", callsign, destination, active_runway);
        Ok(())
    }

    fn lookup_runway_endpoint(&self, airport: &str, runway: &str) -> Result<Option<(i32, f64, f64)>> {
        let runway_path = format!("data/Airports/{}/Runway.txt", airport);
        let Ok(content) = std::fs::read_to_string(&runway_path) else {
            return Ok(None);
        };

        for line in content.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with(';') {
                continue;
            }

            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 8 {
                continue;
            }

            let rwy_a = parts[0];
            let rwy_b = parts[1];

            let heading_a = match parts[2].parse::<i32>() {
                Ok(v) => v,
                Err(_) => continue,
            };
            let heading_b = match parts[3].parse::<i32>() {
                Ok(v) => v,
                Err(_) => continue,
            };

            if runway.eq_ignore_ascii_case(rwy_a) {
                if let Ok((lat, lon)) = sf_coords_to_decimal(parts[4], parts[5]) {
                    return Ok(Some((heading_a, lat, lon)));
                }
            }

            if runway.eq_ignore_ascii_case(rwy_b) {
                if let Ok((lat, lon)) = sf_coords_to_decimal(parts[6], parts[7]) {
                    return Ok(Some((heading_b, lat, lon)));
                }
            }
        }

        Ok(None)
    }
    
    /// Update all aircraft positions and states
    fn update_aircraft(&mut self, delta_time: f64) {
        let sim_config = self.sim_config.clone();
        let nav_db = self.nav_db.clone();
        
        // Collect callsigns of aircraft that will be removed
        let removed_callsigns: Vec<String> = self.aircraft
            .iter()
            .filter(|a| a.is_route_complete())
            .map(|a| a.callsign.clone())
            .collect();
        
        // Remove completed aircraft from used callsigns
        for callsign in &removed_callsigns {
            self.used_callsigns.remove(callsign);
            info!("[SIMULATOR] Aircraft {} completed route and removed", callsign);
        }
        
        // Remove aircraft that have completed their routes
        self.aircraft.retain(|a| !a.is_route_complete());
        
        // Update remaining aircraft
        for aircraft in &mut self.aircraft {
            aircraft.update(delta_time, &nav_db, &sim_config);
        }
    }

    /// Create departure spawn timers
    fn create_departure_timers(&self) -> Vec<(String, u64, u64)> {
        self.scenario.departure_configs()
            .iter()
            .map(|dep| {
                let interval_ticks = (dep.interval as f64 / (1.0 / self.sim_config.radar_update_rate)) as u64;
                (dep.departing.clone(), interval_ticks, 0u64)
            })
            .collect()
    }

    /// Create transit spawn timers
    fn create_transit_timers(&self) -> Vec<(usize, u64, u64)> {
        self.scenario.transit_configs()
            .iter()
            .enumerate()
            .map(|(idx, transit)| {
                let interval_ticks = (transit.interval as f64 / (1.0 / self.sim_config.radar_update_rate)) as u64;
                (idx, interval_ticks, 0u64)
            })
            .collect()
    }

    /// Check and spawn departures
    async fn check_departure_spawns(&mut self, timers: &mut [(String, u64, u64)], loop_count: u64) -> Result<()> {
        for (aerodrome, interval, last_spawn) in timers.iter_mut() {
            if loop_count - *last_spawn >= *interval {
                *last_spawn = loop_count;
                
                if let Some(route) = self.scenario.random_departure_route(aerodrome) {
                    let departure = aerodrome.clone();
                    let arrival = route.arriving.clone();
                    let route_str = route.route.clone();
                    self.spawn_departure(&departure, &arrival, &route_str).await?;
                }
            }
        }
        Ok(())
    }
    
    /// Spawn a departure aircraft
    async fn spawn_departure(&mut self, departure: &str, arrival: &str, route: &str) -> Result<()> {
        // Get airport coordinates
        let airport_coords = self.get_airport_coords(departure)?;
        
        // Get runway information
        let runway = match self.scenario.active_runway(departure) {
            Some(r) => r.to_string(),
            None => return Err(anyhow::anyhow!("No active runway for {}", departure)),
        };
        
        // Parse runway heading (e.g., "27R" -> 270 degrees)
        let runway_heading = self.parse_runway_heading(&runway);
        
        // Generate callsign
        let callsign = self.generate_callsign(departure)?;
        
        // Select aircraft type
        let aircraft_type = self.select_aircraft_type(departure)?;
        
        // Assign squawk
        let squawk = self.assign_squawk();
        
        // Create aircraft
        let aircraft = Aircraft::new_departure(
            callsign.clone(),
            aircraft_type.clone(),
            squawk.clone(),
            departure.to_string(),
            arrival.to_string(),
            route.to_string(),
            self.get_cruise_altitude(route),
            runway,
            airport_coords,
            runway_heading,
        );
        
        info!("[SIMULATOR] Spawned departure {} ({}) from {} to {} via {}", 
              callsign, aircraft.aircraft_type, departure, arrival, 
              aircraft.current_fix().unwrap_or("route"));
        
        // Get flight plan before moving aircraft
        let flight_plan_str = aircraft.flight_plan.to_fsd_string();
        
        // Login pilot to FSD server and send flight plan
        self.login_pilot(&callsign, &aircraft_type, &squawk, &flight_plan_str).await?;
        
        // Send initial position immediately after login
        if let Some(pilot) = self.pilot_clients.get_mut(&callsign) {
            pilot.send_position(
                aircraft.latitude,
                aircraft.longitude,
                aircraft.altitude,
                aircraft.ground_speed,
                aircraft.heading,
                &aircraft.squawk
            ).await?;
        }
        
        // Mark callsign as used
        self.used_callsigns.insert(callsign.clone());

        // Mirror legacy behavior: master controller publishes assigned squawk.
        let (master_callsign, _) = self.scenario.master_controller();
        let bc_message = format!("$CQ{}:@94835:BC:{}:{}", master_callsign, callsign, squawk);
        if let Some(master_controller) = self.ai_controllers.first() {
            if let Err(e) = master_controller.send_message(&bc_message) {
                warn!("[SIMULATOR] Failed to queue BC message for {}: {}", callsign, e);
            }
        }
        
        self.aircraft.push(aircraft);
        
        Ok(())
    }
    
    /// Login a pilot client to the FSD server
    async fn login_pilot(&mut self, callsign: &str, aircraft_type: &str, squawk: &str, flight_plan: &str) -> Result<()> {
        let mut pilot = AiPilot::new(callsign.to_string());
        pilot.connect(&self.server_addr).await?;
        pilot.login(aircraft_type, squawk).await?;
        
        // Send flight plan
        pilot.send_flight_plan(flight_plan).await?;
        
        self.pilot_clients.insert(callsign.to_string(), pilot);
        Ok(())
    }
    
    /// Broadcast all pilot positions to FSD server
    async fn broadcast_pilot_positions(&mut self) -> Result<()> {
        let mut disconnected = Vec::new();
        
        for aircraft in &self.aircraft {
            if let Some(pilot) = self.pilot_clients.get_mut(&aircraft.callsign) {
                if let Err(e) = pilot.send_position(
                    aircraft.latitude,
                    aircraft.longitude,
                    aircraft.altitude,
                    aircraft.ground_speed,
                    aircraft.heading,
                    &aircraft.squawk
                ).await {
                    disconnected.push(aircraft.callsign.clone());
                }
            }
        }
        
        // Remove disconnected pilots
        for callsign in disconnected {
            self.pilot_clients.remove(&callsign);
        }
        
        Ok(())
    }
    
    /// Get airport coordinates from navigation database
    fn get_airport_coords(&self, icao: &str) -> Result<(f64, f64)> {
        // Try to find airport in fix database
        if let Some(coords) = self.nav_db.get(icao) {
            return Ok(*coords);
        }
        
        // Default coordinates for common UK airports
        let coords = match icao {
            "EGSS" => (51.885, 0.235),   // Stansted
            "EGGW" => (51.875, -0.368),  // Luton
            "EGLC" => (51.505, 0.055),   // London City
            "EGLL" => (51.471, -0.461),  // Heathrow
            "EGKK" => (51.148, -0.190),  // Gatwick
            _ => return Err(anyhow::anyhow!("Unknown airport: {}", icao)),
        };
        
        Ok(coords)
    }
    
    /// Parse runway heading from runway identifier
    fn parse_runway_heading(&self, runway: &str) -> i32 {
        // Extract numeric part (e.g., "27R" -> 27)
        let numeric: String = runway.chars().filter(|c| c.is_numeric()).collect();
        if let Ok(rwy_num) = numeric.parse::<i32>() {
            rwy_num * 10
        } else {
            0
        }
    }
    
    /// Generate a unique callsign for an aircraft
    fn generate_callsign(&mut self, departure: &str) -> Result<String> {
        let mut rng = rand::thread_rng();
        
        // Get airline for this airport
        let airlines = self.fleet_config.airports.get(departure)
            .ok_or_else(|| anyhow::anyhow!("No airlines configured for {}", departure))?;
        
        // Try up to 100 times to generate a unique callsign
        for _ in 0..100 {
            let airline = airlines.get(rng.gen_range(0..airlines.len()))
                .ok_or_else(|| anyhow::anyhow!("No airline selected"))?;
            
            // Generate flight number
            let flight_num = rng.gen_range(1..9999);
            let callsign = format!("{}{:04}", airline, flight_num);
            
            // Check if callsign is unique
            if !self.used_callsigns.contains(&callsign) {
                return Ok(callsign);
            }
        }
        
        Err(anyhow::anyhow!("Failed to generate unique callsign after 100 attempts"))
    }
    
    /// Select an aircraft type for departure
    fn select_aircraft_type(&self, departure: &str) -> Result<String> {
        let mut rng = rand::thread_rng();
        
        // Get airlines for this airport
        let airlines = self.fleet_config.airports.get(departure)
            .ok_or_else(|| anyhow::anyhow!("No airlines for {}", departure))?;
        
        let airline = airlines.get(rng.gen_range(0..airlines.len()))
            .ok_or_else(|| anyhow::anyhow!("No airline selected"))?;
        
        // Get aircraft types for this airline
        let aircraft_types = self.fleet_config.airlines.get(airline);
        
        if aircraft_types.is_none() || aircraft_types.unwrap().is_empty() {
            warn!("[SIMULATOR] No aircraft types configured for airline {}, using default A320", airline);
            return Ok("A320".to_string());
        }
        
        let aircraft_types = aircraft_types.unwrap();
        let aircraft_type = aircraft_types.get(rng.gen_range(0..aircraft_types.len()))
            .ok_or_else(|| anyhow::anyhow!("No aircraft type selected"))?;
        
        Ok(aircraft_type.clone())
    }
    
    /// Assign a squawk code
    fn assign_squawk(&mut self) -> String {
        if let Some(squawk) = self.squawk_pool.pop() {
            format!("{:04}", squawk)
        } else {
            // Fallback if pool is empty
            let mut rng = rand::thread_rng();
            format!("{:04}", rng.gen_range(2000..7777))
        }
    }
    
    /// Extract cruise altitude from route
    fn get_cruise_altitude(&self, route: &str) -> u32 {
        // Look for FL in route (e.g., FL350)
        if let Some(fl_pos) = route.find("FL") {
            let fl_str = &route[fl_pos+2..];
            if let Some(num_end) = fl_str.find(|c: char| !c.is_numeric()) {
                if let Ok(fl) = fl_str[..num_end].parse::<u32>() {
                    return fl;
                }
            }
        }
        
        // Default cruise altitude
        360
    }

    /// Check and spawn transits
    async fn check_transit_spawns(&self, timers: &mut [(usize, u64, u64)], loop_count: u64) -> Result<()> {
        for (idx, interval, last_spawn) in timers.iter_mut() {
            if loop_count - *last_spawn >= *interval {
                *last_spawn = loop_count;
                
                if let Some(route) = self.scenario.random_transit_route(*idx) {
                    info!("[SIMULATOR] Spawning transit: {} -> {} at FL{:03} via {}", 
                          route.departing, route.arriving, route.current_level / 100, route.route);
                    // TODO: Create and spawn aircraft
                }
            }
        }
        Ok(())
    }

    /// Stop the simulation
    pub async fn stop(&mut self) -> Result<()> {
        info!("[SIMULATOR] Stopping simulation...");
        self.running = false;
        
        // Disconnect all pilots
        for (callsign, mut pilot) in self.pilot_clients.drain() {
            info!("[SIMULATOR] Disconnecting pilot {}", callsign);
            pilot.disconnect().await?;
        }
        
        // Disconnect all AI controllers
        for controller in &mut self.ai_controllers {
            controller.disconnect().await?;
        }
        
        self.ai_controllers.clear();
        
        info!("[SIMULATOR] Simulation stopped");
        Ok(())
    }

    /// Get simulation statistics
    pub fn statistics(&self) -> SimulatorStats {
        SimulatorStats {
            running: self.running,
            active_controllers: self.ai_controllers.len(),
            active_pilots: 0, // TODO: Track pilots
            scenario_name: self.scenario.name.clone(),
        }
    }
}

/// Statistics about the running simulator
#[derive(Debug, Clone)]
pub struct SimulatorStats {
    pub running: bool,
    pub active_controllers: usize,
    pub active_pilots: usize,
    pub scenario_name: String,
}

impl std::fmt::Display for SimulatorStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Simulator Status:")?;
        writeln!(f, "  Scenario: {}", self.scenario_name)?;
        writeln!(f, "  Running: {}", self.running)?;
        writeln!(f, "  Active Controllers: {}", self.active_controllers)?;
        writeln!(f, "  Active Pilots: {}", self.active_pilots)?;
        Ok(())
    }
}
