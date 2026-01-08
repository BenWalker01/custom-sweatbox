use anyhow::{Result, Context};
use std::path::Path;
use crate::config::{ProfileConfig, DepartureRoute, StandardDeparture, TransitRoute, StandardTransit};
use rand::seq::SliceRandom;
use rand::Rng;

/// Represents a loaded scenario with utility methods for simulation
#[derive(Debug, Clone)]
pub struct Scenario {
    pub config: ProfileConfig,
    pub name: String,
}

impl Scenario {
    /// Load a scenario from a JSON file
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self> {
        let path_ref = path.as_ref();
        let config = ProfileConfig::load(
            path_ref.to_str()
                .ok_or_else(|| anyhow::anyhow!("Invalid path"))?
        )?;
        
        let name = path_ref
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("Unknown")
            .to_string();
        
        Ok(Self { config, name })
    }

    /// Get all active aerodromes
    pub fn active_aerodromes(&self) -> &[String] {
        &self.config.active_aerodromes
    }

    /// Get the active runway for a specific aerodrome
    pub fn active_runway(&self, aerodrome: &str) -> Option<&str> {
        self.config.active_runways.get(aerodrome).map(|s| s.as_str())
    }

    /// Get all departure configurations
    pub fn departure_configs(&self) -> &[StandardDeparture] {
        &self.config.std_departures
    }

    /// Get all transit configurations
    pub fn transit_configs(&self) -> &[StandardTransit] {
        &self.config.std_transits
    }

    /// Get a random departure route for a specific aerodrome
    pub fn random_departure_route(&self, aerodrome: &str) -> Option<&DepartureRoute> {
        let mut rng = rand::thread_rng();
        
        self.config.std_departures
            .iter()
            .find(|d| d.departing == aerodrome)
            .and_then(|d| d.routes.choose(&mut rng))
    }

    /// Get a random transit route from a specific configuration index
    pub fn random_transit_route(&self, transit_index: usize) -> Option<&TransitRoute> {
        let mut rng = rand::thread_rng();
        
        self.config.std_transits
            .get(transit_index)
            .and_then(|t| t.routes.choose(&mut rng))
    }

    /// Get all departure aerodromes
    pub fn departure_aerodromes(&self) -> Vec<&str> {
        self.config.std_departures
            .iter()
            .map(|d| d.departing.as_str())
            .collect()
    }

    /// Get departure interval for a specific aerodrome
    pub fn departure_interval(&self, aerodrome: &str) -> Option<u64> {
        self.config.std_departures
            .iter()
            .find(|d| d.departing == aerodrome)
            .map(|d| d.interval)
    }

    /// Get all transit intervals
    pub fn transit_intervals(&self) -> Vec<u64> {
        self.config.std_transits
            .iter()
            .map(|t| t.interval)
            .collect()
    }

    /// Get master controller information
    pub fn master_controller(&self) -> (&str, &str) {
        (&self.config.master_controller, &self.config.master_controller_freq)
    }

    /// Get all active controller positions
    pub fn active_controllers(&self) -> &[String] {
        &self.config.active_controllers
    }

    /// Get other controller positions
    pub fn other_controllers(&self) -> &[(String, String)] {
        &self.config.other_controllers
    }

    /// Check if a specific controller is active
    pub fn is_controller_active(&self, controller: &str) -> bool {
        self.config.active_controllers.iter().any(|c| c == controller)
    }

    /// Get all unique arriving aerodromes from departures
    pub fn departure_destinations(&self) -> Vec<&str> {
        let mut destinations: Vec<&str> = self.config.std_departures
            .iter()
            .flat_map(|d| d.routes.iter().map(|r| r.arriving.as_str()))
            .collect();
        destinations.sort_unstable();
        destinations.dedup();
        destinations
    }

    /// Get all unique arriving aerodromes from transits
    pub fn transit_destinations(&self) -> Vec<&str> {
        let mut destinations: Vec<&str> = self.config.std_transits
            .iter()
            .flat_map(|t| t.routes.iter().map(|r| r.arriving.as_str()))
            .collect();
        destinations.sort_unstable();
        destinations.dedup();
        destinations
    }

    /// Get all unique departing aerodromes from transits
    pub fn transit_origins(&self) -> Vec<&str> {
        let mut origins: Vec<&str> = self.config.std_transits
            .iter()
            .flat_map(|t| t.routes.iter().map(|r| r.departing.as_str()))
            .collect();
        origins.sort_unstable();
        origins.dedup();
        origins
    }

    /// Get statistics about the scenario
    pub fn statistics(&self) -> ScenarioStats {
        let total_departure_routes: usize = self.config.std_departures
            .iter()
            .map(|d| d.routes.len())
            .sum();
        
        let total_transit_routes: usize = self.config.std_transits
            .iter()
            .map(|t| t.routes.len())
            .sum();

        ScenarioStats {
            name: self.name.clone(),
            active_aerodromes: self.config.active_aerodromes.len(),
            departure_configs: self.config.std_departures.len(),
            transit_configs: self.config.std_transits.len(),
            total_departure_routes,
            total_transit_routes,
            active_controllers: self.config.active_controllers.len(),
            other_controllers: self.config.other_controllers.len(),
        }
    }
}

/// Statistics about a loaded scenario
#[derive(Debug, Clone)]
pub struct ScenarioStats {
    pub name: String,
    pub active_aerodromes: usize,
    pub departure_configs: usize,
    pub transit_configs: usize,
    pub total_departure_routes: usize,
    pub total_transit_routes: usize,
    pub active_controllers: usize,
    pub other_controllers: usize,
}

impl std::fmt::Display for ScenarioStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Scenario: {}", self.name)?;
        writeln!(f, "  Active Aerodromes: {}", self.active_aerodromes)?;
        writeln!(f, "  Departure Configurations: {} ({} routes)", 
                 self.departure_configs, self.total_departure_routes)?;
        writeln!(f, "  Transit Configurations: {} ({} routes)", 
                 self.transit_configs, self.total_transit_routes)?;
        writeln!(f, "  Active Controllers: {}", self.active_controllers)?;
        writeln!(f, "  Other Controllers: {}", self.other_controllers)?;
        Ok(())
    }
}

/// Builder for creating scenarios programmatically (for testing)
#[derive(Debug, Default)]
pub struct ScenarioBuilder {
    active_aerodromes: Vec<String>,
    active_runways: std::collections::HashMap<String, String>,
    active_controllers: Vec<String>,
    master_controller: String,
    master_controller_freq: String,
    other_controllers: Vec<(String, String)>,
    std_departures: Vec<StandardDeparture>,
    std_transits: Vec<StandardTransit>,
}

impl ScenarioBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_aerodrome(mut self, icao: String, runway: String) -> Self {
        self.active_aerodromes.push(icao.clone());
        self.active_runways.insert(icao, runway);
        self
    }

    pub fn master_controller(mut self, callsign: String, freq: String) -> Self {
        self.master_controller = callsign;
        self.master_controller_freq = freq;
        self
    }

    pub fn add_controller(mut self, callsign: String) -> Self {
        self.active_controllers.push(callsign);
        self
    }

    pub fn add_other_controller(mut self, callsign: String, freq: String) -> Self {
        self.other_controllers.push((callsign, freq));
        self
    }

    pub fn add_departure_config(mut self, config: StandardDeparture) -> Self {
        self.std_departures.push(config);
        self
    }

    pub fn add_transit_config(mut self, config: StandardTransit) -> Self {
        self.std_transits.push(config);
        self
    }

    pub fn build(self) -> Scenario {
        Scenario {
            name: "Built Scenario".to_string(),
            config: ProfileConfig {
                active_aerodromes: self.active_aerodromes,
                active_runways: self.active_runways,
                active_controllers: self.active_controllers,
                master_controller: self.master_controller,
                master_controller_freq: self.master_controller_freq,
                other_controllers: self.other_controllers,
                inactive_sectors: vec![],
                std_departures: self.std_departures,
                std_transits: self.std_transits,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_load_scenario() -> Result<()> {
        let scenario = Scenario::load("profiles/TCE + TCNE.json")?;
        
        assert_eq!(scenario.active_aerodromes().len(), 4);
        assert!(scenario.active_aerodromes().contains(&"EGSS".to_string()));
        assert!(scenario.active_aerodromes().contains(&"EGGW".to_string()));
        assert!(scenario.active_aerodromes().contains(&"EGLC".to_string()));
        assert!(scenario.active_aerodromes().contains(&"EGLL".to_string()));
        
        Ok(())
    }

    #[test]
    fn test_scenario_stats() -> Result<()> {
        let scenario = Scenario::load("profiles/TCE + TCNE.json")?;
        let stats = scenario.statistics();
        
        println!("{}", stats);
        
        assert_eq!(stats.active_aerodromes, 4);
        assert!(stats.total_departure_routes > 0);
        assert!(stats.total_transit_routes > 0);
        
        Ok(())
    }

    #[test]
    fn test_random_routes() -> Result<()> {
        let scenario = Scenario::load("profiles/TCE + TCNE.json")?;
        
        // Test random departure route
        let dep_route = scenario.random_departure_route("EGSS");
        assert!(dep_route.is_some());
        
        if let Some(route) = dep_route {
            println!("Random departure from EGSS: {} -> {}", route.route, route.arriving);
        }
        
        // Test random transit route
        let transit_route = scenario.random_transit_route(0);
        assert!(transit_route.is_some());
        
        if let Some(route) = transit_route {
            println!("Random transit: {} -> {} via {}", 
                     route.departing, route.arriving, route.route);
        }
        
        Ok(())
    }

    #[test]
    fn test_departure_intervals() -> Result<()> {
        let scenario = Scenario::load("profiles/TCE + TCNE.json")?;
        
        let egss_interval = scenario.departure_interval("EGSS");
        assert_eq!(egss_interval, Some(180));
        
        let eggw_interval = scenario.departure_interval("EGGW");
        assert_eq!(eggw_interval, Some(180));
        
        let eglc_interval = scenario.departure_interval("EGLC");
        assert_eq!(eglc_interval, Some(400));
        
        let egll_interval = scenario.departure_interval("EGLL");
        assert_eq!(egll_interval, Some(200));
        
        Ok(())
    }

    #[test]
    fn test_active_runways() -> Result<()> {
        let scenario = Scenario::load("profiles/TCE + TCNE.json")?;
        
        assert_eq!(scenario.active_runway("EGSS"), Some("22"));
        assert_eq!(scenario.active_runway("EGGW"), Some("25"));
        assert_eq!(scenario.active_runway("EGLC"), Some("27"));
        assert_eq!(scenario.active_runway("EGLL"), Some("27R"));
        
        Ok(())
    }

    #[test]
    fn test_controllers() -> Result<()> {
        let scenario = Scenario::load("profiles/TCE + TCNE.json")?;
        
        let (master, freq) = scenario.master_controller();
        assert_eq!(master, "LON_E_CTR");
        assert_eq!(freq, "18480");
        
        assert!(scenario.is_controller_active("LTC_E_CTR"));
        assert!(scenario.is_controller_active("LTC_N_CTR"));
        assert!(scenario.is_controller_active("ESSEX_APP"));
        
        Ok(())
    }

    #[test]
    fn test_destinations() -> Result<()> {
        let scenario = Scenario::load("profiles/TCE + TCNE.json")?;
        
        let dep_destinations = scenario.departure_destinations();
        println!("Departure destinations: {:?}", dep_destinations);
        assert!(dep_destinations.contains(&"EHAM"));
        assert!(dep_destinations.contains(&"EDDF"));
        
        let transit_destinations = scenario.transit_destinations();
        println!("Transit destinations: {:?}", transit_destinations);
        assert!(transit_destinations.contains(&"EGKK"));
        assert!(transit_destinations.contains(&"EGSS"));
        
        let transit_origins = scenario.transit_origins();
        println!("Transit origins: {:?}", transit_origins);
        assert!(transit_origins.contains(&"EHAM"));
        assert!(transit_origins.contains(&"EBBR"));
        
        Ok(())
    }

    #[test]
    fn test_scenario_builder() {
        let scenario = ScenarioBuilder::new()
            .add_aerodrome("EGLL".to_string(), "27L".to_string())
            .master_controller("LON_S_CTR".to_string(), "29430".to_string())
            .add_controller("LON_S_CTR".to_string())
            .build();
        
        assert_eq!(scenario.active_aerodromes().len(), 1);
        assert_eq!(scenario.active_runway("EGLL"), Some("27L"));
        assert_eq!(scenario.master_controller(), ("LON_S_CTR", "29430"));
    }
}
