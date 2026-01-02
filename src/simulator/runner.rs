use crate::config::{ProfileConfig, SimulationConfig, FleetConfig, get_ccams_squawks, DepartureRoute, TransitRoute};
use crate::simulator::{Plane, FlightPlan, Route, ControllerData};
use crate::utils::navigation::FixDatabase;
use crate::utils::performance::PerformanceDatabase;
use std::sync::Arc;
use tokio::time::{interval, Duration};
use tokio::net::TcpStream;
use tokio::io::AsyncWriteExt;
use rand::Rng;
use std::collections::HashSet;

pub struct SimulationRunner {
    pub profile: ProfileConfig,
    pub sim_config: SimulationConfig,
    pub fleet_config: FleetConfig,
    pub fix_db: Arc<FixDatabase>,
    pub perf_db: Arc<PerformanceDatabase>,
    pub planes: Vec<Plane>,
    pub plane_sockets: Vec<TcpStream>,
    pub server_address: String,
    pub used_squawks: HashSet<u16>,
    pub used_callsigns: HashSet<String>,
    pub available_squawks: Vec<u16>,
}

impl SimulationRunner {
    pub fn new(
        profile: ProfileConfig,
        sim_config: SimulationConfig,
        fleet_config: FleetConfig,
        fix_db: Arc<FixDatabase>,
        perf_db: Arc<PerformanceDatabase>,
        server_address: String,
    ) -> Self {
        Self {
            profile,
            sim_config,
            fleet_config,
            fix_db,
            perf_db,
            planes: Vec::new(),
            plane_sockets: Vec::new(),
            server_address,
            used_squawks: HashSet::new(),
            used_callsigns: HashSet::new(),
            available_squawks: get_ccams_squawks(),
        }
    }

    /// Generate a unique squawk code
    pub fn generate_squawk(&mut self) -> Option<u16> {
        let mut rng = rand::thread_rng();
        
        for _ in 0..100 {
            let idx = rng.gen_range(0..self.available_squawks.len());
            let squawk = self.available_squawks[idx];
            
            if !self.used_squawks.contains(&squawk) {
                self.used_squawks.insert(squawk);
                return Some(squawk);
            }
        }
        
        None
    }

    /// Generate a unique callsign for an airport
    pub fn generate_callsign(&mut self, airport: &str) -> Option<(String, String)> {
        let mut rng = rand::thread_rng();
        
        // Get airlines that operate at this airport
        let airlines = self.fleet_config.airports.get(airport)?;
        
        for _ in 0..100 {
            let airline = &airlines[rng.gen_range(0..airlines.len())];
            let flight_num = rng.gen_range(1..9999);
            let callsign = format!("{}{}", airline, flight_num);
            
            if !self.used_callsigns.contains(&callsign) {
                // Get aircraft type for this airline
                let aircraft_types = self.fleet_config.airlines.get(airline)?;
                let aircraft_type = aircraft_types[rng.gen_range(0..aircraft_types.len())].clone();
                
                self.used_callsigns.insert(callsign.clone());
                return Some((callsign, aircraft_type));
            }
        }
        
        None
    }

    /// Connect a plane to the FSD server as a pilot
    async fn connect_plane_to_fsd(&mut self, plane: &Plane) -> Result<TcpStream, std::io::Error> {
        let mut stream = TcpStream::connect(&self.server_address).await?;
        
        // Send FSD pilot login messages
        let login_msg = format!(
            "#AA{}:SERVER:PILOT:{}:1:100:1:{}:{}:{}\r\n",
            plane.callsign,
            plane.callsign,
            plane.lat,
            plane.lon,
            plane.altitude as i32
        );
        
        stream.write_all(login_msg.as_bytes()).await?;
        
        // Send flight plan
        let fp_msg = format!(
            "$FP{}:*A:{}:{}:{}:{}:{}:{}:{}:0:0:0:0:::::::\r\n",
            plane.callsign,
            plane.flight_plan.flight_rules,
            plane.flight_plan.aircraft_type,
            plane.flight_plan.enroute_speed,
            plane.flight_plan.departure,
            plane.flight_plan.cruise_altitude,
            plane.flight_plan.destination,
            plane.flight_plan.route.route_string
        );
        
        stream.write_all(fp_msg.as_bytes()).await?;
        
        println!("  → Connected {} to FSD server", plane.callsign);
        
        Ok(stream)
    }

    /// Send position update for all planes
    async fn send_position_updates(&mut self) {
        for (i, plane) in self.planes.iter().enumerate() {
            if let Some(stream) = self.plane_sockets.get_mut(i) {
                let pos_msg = plane.position_update_text();
                if let Err(e) = stream.write_all(format!("{}\r\n", pos_msg).as_bytes()).await {
                    eprintln!("Failed to send position for {}: {}", plane.callsign, e);
                }
            }
        }
    }

    /// Spawn a departure aircraft
    pub async fn spawn_departure(&mut self, dep_route: &DepartureRoute, departing: &str) {
        let runway = match self.profile.active_runways.get(departing) {
            Some(rwy) => rwy.clone(),
            None => {
                eprintln!("No active runway for {}", departing);
                return;
            }
        };

        let (callsign, aircraft_type) = match self.generate_callsign(departing) {
            Some(cs) => cs,
            None => {
                eprintln!("Failed to generate callsign for {}", departing);
                return;
            }
        };

        let squawk = match self.generate_squawk() {
            Some(sq) => sq,
            None => {
                eprintln!("Failed to generate squawk");
                return;
            }
        };

        // Expand SID in route
        let mut route = Route::new(
            dep_route.route.clone(),
            departing.to_string(),
            Some(dep_route.arriving.clone()),
        );
        if let Err(e) = route.expand_sid(departing, &runway, &self.fix_db) {
            eprintln!("Failed to expand SID: {}", e);
        }

        // Determine cruise level based on destination
        let cruise_level = if dep_route.arriving.starts_with("EG") {
            25000
        } else {
            37000
        };

        let flight_plan = FlightPlan::new(
            "I".to_string(),
            aircraft_type.clone(),
            250,
            departing.to_string(),
            1130,
            1130,
            cruise_level,
            dep_route.arriving.clone(),
            route,
        );

        // Get runway coordinates - simplified for now, use airport center
        let (lat, lon) = self.fix_db.get(departing)
            .copied()
            .unwrap_or((51.15487, -0.16454));

        // Get runway heading (simplified - just use a default)
        let heading = runway.parse::<u32>()
            .map(|rwy_num| (rwy_num * 10) as f64)
            .unwrap_or(270.0);

        // Create the plane
        let mut plane = Plane::new(
            callsign.clone(),
            squawk as u32,
            self.sim_config.airport_elevations.get(departing).copied().unwrap_or(0) as f64,
            heading,
            0.0, // Starting stationary
            lat,
            lon,
            0.0,
            crate::simulator::plane_mode::PlaneMode::FlightPlan,
            flight_plan,
            None,
            None,
            None,
        );

        // Set databases
        plane.set_fix_database(Arc::clone(&self.fix_db));
        plane.set_performance_database(Arc::clone(&self.perf_db));

        // Set initial climb parameters
        plane.target_altitude = 5000.0;
        plane.target_speed = 250.0;

        println!("✓ Spawned departure: {} ({}) at {}/{} -> {}",
                 callsign, aircraft_type, departing, runway, dep_route.arriving);

        // Connect to FSD server
        match self.connect_plane_to_fsd(&plane).await {
            Ok(stream) => {
                self.planes.push(plane);
                self.plane_sockets.push(stream);
            }
            Err(e) => {
                eprintln!("  ✗ Failed to connect {} to FSD: {}", callsign, e);
            }
        }
    }

    /// Spawn a transit/arrival aircraft
    pub async fn spawn_transit(&mut self, transit: &TransitRoute) {
        let (callsign, aircraft_type) = match self.generate_callsign(&transit.departing) {
            Some(cs) => cs,
            None => {
                eprintln!("Failed to generate callsign for transit from {}", transit.departing);
                return;
            }
        };

        let squawk = match self.generate_squawk() {
            Some(sq) => sq,
            None => {
                eprintln!("Failed to generate squawk for transit");
                return;
            }
        };

        // Parse route to get first fix
        let first_fix = transit.route.split_whitespace()
            .next()
            .unwrap_or("UNKNOWN")
            .to_string();

        // Create route with STAR expansion
        let mut route = Route::new(
            transit.route.clone(),
            transit.departing.clone(),
            Some(transit.arriving.clone()),
        );
        
        // Expand STAR if route ends with one
        if let Some(runway) = self.profile.active_runways.get(&transit.arriving) {
            if let Err(e) = route.expand_star(&transit.arriving, runway, &self.fix_db) {
                eprintln!("Failed to expand STAR: {}", e);
            }
        }

        // Determine speed based on altitude
        let speed = if transit.current_level > 30000 {
            450.0
        } else if transit.current_level > 10000 {
            350.0
        } else {
            250.0
        };

        let flight_plan = FlightPlan::new(
            "I".to_string(),
            aircraft_type.clone(),
            250,
            transit.departing.clone(),
            1130,
            1130,
            transit.cruise_level,
            transit.arriving.clone(),
            route,
        );

        // Get heading (default for now)
        let heading = 270.0;

        // Create transit plane using from_fix
        let controller_data = Some(ControllerData {
            controller: self.profile.master_controller.clone(),
            release_point: first_fix.clone(),
        });

        let mut plane = Plane::from_fix(
            callsign.clone(),
            first_fix.clone(),
            squawk as u32,
            transit.current_level as f64,
            heading,
            speed,
            0.0, // Level flight initially
            flight_plan,
            controller_data,
            Some(transit.first_controller.clone()),
            Some(Arc::clone(&self.fix_db)),
        );

        plane.set_performance_database(Arc::clone(&self.perf_db));
        plane.target_altitude = transit.current_level as f64;
        plane.target_speed = speed;

        println!("✓ Spawned transit: {} ({}) from {} at FL{} -> {}",
                 callsign, aircraft_type, transit.departing,
                 transit.current_level / 100, transit.arriving);

        // Connect to FSD server
        match self.connect_plane_to_fsd(&plane).await {
            Ok(stream) => {
                self.planes.push(plane);
                self.plane_sockets.push(stream);
            }
            Err(e) => {
                eprintln!("  ✗ Failed to connect {} to FSD: {}", callsign, e);
            }
        }
    }

    /// Run the simulation
    pub async fn run(runner: Arc<tokio::sync::RwLock<Self>>) {
        {
            let r = runner.read().await;
            println!("=== Starting Simulation ===");
            println!("Profile: {} departures, {} transits",
                     r.profile.std_departures.len(),
                     r.profile.std_transits.len());
            println!("Active airports: {:?}", r.profile.active_aerodromes);
            println!("Master controller: {} on {}",
                     r.profile.master_controller,
                     r.profile.master_controller_freq);
            println!();
        }

        // Spawn initial aircraft
        {
            let mut r = runner.write().await;
            // Spawn one from each departure config
            for dep_config in r.profile.std_departures.clone() {
                if !dep_config.routes.is_empty() {
                    let route = &dep_config.routes[rand::thread_rng().gen_range(0..dep_config.routes.len())];
                    r.spawn_departure(route, &dep_config.departing).await;
                }
            }

            // Spawn one from each transit config
            for transit_config in r.profile.std_transits.clone() {
                if !transit_config.routes.is_empty() {
                    let route = &transit_config.routes[rand::thread_rng().gen_range(0..transit_config.routes.len())];
                    r.spawn_transit(route).await;
                }
            }
        }

        // Start continuous spawn timers
        let runner_clone = Arc::clone(&runner);
        tokio::spawn(async move {
            Self::spawn_loop(runner_clone).await;
        });

        // Main position update loop
        let radar_rate = {
            let r = runner.read().await;
            r.sim_config.radar_update_rate / r.sim_config.time_multiplier
        };

        let mut update_interval = interval(Duration::from_secs_f64(radar_rate));

        {
            let r = runner.read().await;
            println!("Simulation running with {} initial aircraft", r.planes.len());
            println!("Radar update rate: {} seconds", radar_rate);
            println!();
        }
        
        // Main loop - runs indefinitely
        loop {
            update_interval.tick().await;
            let mut r = runner.write().await;
            r.update_positions();
            r.send_position_updates().await;
        }
    }

    /// Continuous spawn loop
    async fn spawn_loop(runner: Arc<tokio::sync::RwLock<Self>>) {
        let (dep_configs, transit_configs) = {
            let r = runner.read().await;
            (r.profile.std_departures.clone(), r.profile.std_transits.clone())
        };

        // Spawn departure timers
        for dep_config in dep_configs {
            let runner_clone = Arc::clone(&runner);
            tokio::spawn(async move {
                let mut spawn_interval = interval(Duration::from_secs(dep_config.interval));
                spawn_interval.tick().await; // Skip first tick since we spawned initially
                
                loop {
                    spawn_interval.tick().await;
                    
                    if !dep_config.routes.is_empty() {
                        let route = dep_config.routes[rand::thread_rng().gen_range(0..dep_config.routes.len())].clone();
                        let departing = dep_config.departing.clone();
                        
                        let mut r = runner_clone.write().await;
                        r.spawn_departure(&route, &departing).await;
                    }
                }
            });
        }

        // Spawn transit timers
        for transit_config in transit_configs {
            let runner_clone = Arc::clone(&runner);
            tokio::spawn(async move {
                let mut spawn_interval = interval(Duration::from_secs(transit_config.interval));
                spawn_interval.tick().await; // Skip first tick since we spawned initially
                
                loop {
                    spawn_interval.tick().await;
                    
                    if !transit_config.routes.is_empty() {
                        let route = transit_config.routes[rand::thread_rng().gen_range(0..transit_config.routes.len())].clone();
                        
                        let mut r = runner_clone.write().await;
                        r.spawn_transit(&route).await;
                    }
                }
            });
        }
    }

    /// Update all aircraft positions
    fn update_positions(&mut self) {
        println!("=== Position Update ===");
        if self.planes.is_empty() {
            println!("No aircraft active");
        }
        
        for plane in &mut self.planes {
            plane.calculate_position();
            println!("{}: {} at {:.4}°, {:.4}° FL{} {} kts",
                     plane.callsign,
                     plane.mode,
                     plane.lat,
                     plane.lon,
                     (plane.altitude / 100.0) as i32,
                     plane.speed as i32);
        }
        println!();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::navigation::load_navigation_data;
    use crate::utils::performance::load_performance_data;

    #[tokio::test]
    async fn test_simulation_runner() {
        let profile = ProfileConfig::load("profiles/TCE + TCNE.json").unwrap();
        let sim_config = SimulationConfig::default();
        let fleet_config = FleetConfig::default();
        
        let fix_db = Arc::new(load_navigation_data("data").unwrap());
        let perf_db = Arc::new(load_performance_data("_data/AircraftPerformace.txt").unwrap());
        
        let mut runner = SimulationRunner::new(
            profile,
            sim_config,
            fleet_config,
            fix_db,
            perf_db,
        );
        
        // Test spawning a few aircraft
        runner.run().await;
        
        assert!(!runner.planes.is_empty());
    }
}
