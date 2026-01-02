use crate::config::{ProfileConfig, SimulationConfig, FleetConfig, get_ccams_squawks, DepartureRoute, TransitRoute};
use crate::simulator::{Plane, FlightPlan, Route};
use crate::utils::navigation::FixDatabase;
use crate::utils::performance::PerformanceDatabase;
use std::sync::Arc;
use tokio::time::{interval, Duration};
use rand::Rng;
use std::collections::HashSet;

pub struct SimulationRunner {
    pub profile: ProfileConfig,
    pub sim_config: SimulationConfig,
    pub fleet_config: FleetConfig,
    pub fix_db: Arc<FixDatabase>,
    pub perf_db: Arc<PerformanceDatabase>,
    pub planes: Vec<Plane>,
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
    ) -> Self {
        Self {
            profile,
            sim_config,
            fleet_config,
            fix_db,
            perf_db,
            planes: Vec::new(),
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

    /// Spawn a departure aircraft
    pub fn spawn_departure(&mut self, dep_route: &DepartureRoute, departing: &str) {
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

        // Create flight plan with SID-enhanced route
        let route = Route::new(
            dep_route.route.clone(),
            departing.to_string(),
            Some(dep_route.arriving.clone()),
        );

        let flight_plan = FlightPlan::new(
            "I".to_string(),
            aircraft_type.clone(),
            250,
            departing.to_string(),
            1130,
            1130,
            37000,
            dep_route.arriving.clone(),
            route,
        );

        // For now, just create a simple departure at runway
        // In full implementation, would use Plane::departure properly
        println!("Spawned departure: {} ({}) at {}/{} -> {}", 
                 callsign, aircraft_type, departing, runway, dep_route.arriving);

        // Simplified spawn - would need proper Plane::departure signature
        // self.planes.push(plane);
    }

    /// Spawn a transit/arrival aircraft
    pub fn spawn_transit(&mut self, transit: &TransitRoute) {
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
        let route = Route::new(
            transit.route.clone(),
            transit.departing.clone(),
            Some(transit.arriving.clone()),
        );

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

        // Simplification: spawn aircraft at first fix
        // In full implementation, would use Plane::from_fix with proper arguments
        println!("Spawned transit: {} ({}) at {} FL{} -> {} FL{}", 
                 callsign, aircraft_type, first_fix, 
                 transit.current_level / 100, transit.arriving,
                 transit.cruise_level / 100);

        // self.planes.push(plane);
    }

    /// Run the simulation
    pub async fn run(&mut self) {
        println!("=== Starting Simulation ===");
        println!("Profile: {} departures, {} transits",
                 self.profile.std_departures.len(),
                 self.profile.std_transits.len());
        println!("Active airports: {:?}", self.profile.active_aerodromes);
        println!("Master controller: {} on {}",
                 self.profile.master_controller,
                 self.profile.master_controller_freq);
        println!();

        // Start departure spawners
        for dep_config in &self.profile.std_departures.clone() {
            let dep_config = dep_config.clone();
            let interval_secs = dep_config.interval;
            
            // Spawn immediately, then at intervals
            if !dep_config.routes.is_empty() {
                let route = &dep_config.routes[rand::thread_rng().gen_range(0..dep_config.routes.len())];
                self.spawn_departure(route, &dep_config.departing);
            }
            
            // Note: In full implementation, would spawn with tokio::spawn
            // For now, this demonstrates the structure
        }

        // Start transit spawners  
        for transit_config in &self.profile.std_transits.clone() {
            if !transit_config.routes.is_empty() {
                let route = &transit_config.routes[rand::thread_rng().gen_range(0..transit_config.routes.len())];
                self.spawn_transit(route);
            }
        }

        // Position update loop
        let mut update_interval = interval(Duration::from_secs_f64(
            self.sim_config.radar_update_rate / self.sim_config.time_multiplier
        ));

        println!("Simulation running with {} initial aircraft", self.planes.len());
        println!("Radar update rate: {} seconds", self.sim_config.radar_update_rate);
        
        // In full implementation, this would run indefinitely
        // and update plane positions, handle messages, etc.
        for _ in 0..3 {
            update_interval.tick().await;
            self.update_positions();
        }
    }

    /// Update all aircraft positions
    fn update_positions(&mut self) {
        println!("\n=== Position Update ===");
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
        
        let fix_db = Arc::new(load_navigation_data("_old/data").unwrap());
        let perf_db = Arc::new(load_performance_data("_old/data/AircraftPerformace.txt").unwrap());
        
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
