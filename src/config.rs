use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use anyhow::{Result, Context};

/// Configuration for a single departure route
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DepartureRoute {
    pub route: String,
    pub arriving: String,
}

/// Configuration for standard departures from an airport
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StandardDeparture {
    pub departing: String,
    pub interval: u64, // seconds between spawns
    pub routes: Vec<DepartureRoute>,
}

/// Configuration for a transit route
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TransitRoute {
    pub departing: String,
    pub arriving: String,
    pub current_level: u32,
    pub cruise_level: u32,
    pub route: String,
    pub first_controller: String,
}

/// Configuration for standard transits
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StandardTransit {
    pub interval: u64,
    pub routes: Vec<TransitRoute>,
}

/// Main profile configuration loaded from JSON
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProfileConfig {
    #[serde(default)]
    pub std_departures: Vec<StandardDeparture>,
    #[serde(default)]
    pub std_transits: Vec<StandardTransit>,
    
    // Profile-specific settings
    pub active_aerodromes: Vec<String>,
    pub active_runways: HashMap<String, String>,
    pub active_controllers: Vec<String>,
    pub master_controller: String,
    pub master_controller_freq: String,
    #[serde(default)]
    pub inactive_sectors: Vec<String>,
    #[serde(default)]
    pub other_controllers: Vec<(String, String)>,
}

impl ProfileConfig {
    pub fn load(path: &str) -> Result<Self> {
        let contents = fs::read_to_string(path)
            .with_context(|| format!("Failed to read profile: {}", path))?;
        let config: ProfileConfig = serde_json::from_str(&contents)
            .with_context(|| format!("Failed to parse profile JSON: {}", path))?;
        Ok(config)
    }
}

/// Simulation constants (from Constants.py)
#[derive(Debug, Clone)]
pub struct SimulationConfig {
    pub port: u16,
    pub turn_rate: f64,
    pub taxi_speed: f64,
    pub push_speed: f64,
    pub climb_rate: f64,
    pub descent_rate: f64,
    pub high_descent_rate: f64,
    pub time_multiplier: f64,
    pub radar_update_rate: f64,
    
    pub airport_elevations: HashMap<String, u32>,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        let mut airport_elevations = HashMap::new();
        airport_elevations.insert("EGLL".to_string(), 83);
        airport_elevations.insert("EGKK".to_string(), 202);
        airport_elevations.insert("EGCC".to_string(), 257);
        airport_elevations.insert("EGPH".to_string(), 135);
        airport_elevations.insert("EGNX".to_string(), 306);
        airport_elevations.insert("EGGD".to_string(), 622);
        airport_elevations.insert("EGGW".to_string(), 526);
        airport_elevations.insert("EGSS".to_string(), 348);
        airport_elevations.insert("EGPF".to_string(), 26);
        airport_elevations.insert("EGAA".to_string(), 268);
        airport_elevations.insert("EGAC".to_string(), 50);
        airport_elevations.insert("EGNT".to_string(), 266);
        airport_elevations.insert("EGMC".to_string(), 49);
        airport_elevations.insert("EGNM".to_string(), 681);
        airport_elevations.insert("EGPK".to_string(), 65);

        Self {
            port: 6809,
            turn_rate: 3.0,  // 3 degrees per second (standard rate turn)
            taxi_speed: 15.0,
            push_speed: 5.0,
            climb_rate: 2000.0,  // 2000 ft/min default
            descent_rate: -2000.0,
            high_descent_rate: -3000.0,
            time_multiplier: 1.0,
            radar_update_rate: 5.0,
            airport_elevations,
        }
    }
}

/// Fleet configuration (which airlines fly which aircraft)
#[derive(Debug, Clone)]
pub struct FleetConfig {
    pub airlines: HashMap<String, Vec<String>>,
    pub airports: HashMap<String, Vec<String>>,
}

impl Default for FleetConfig {
    fn default() -> Self {
        let mut airlines = HashMap::new();
        airlines.insert("RYR".to_string(), vec!["B738".to_string(), "B38M".to_string(), "A320".to_string()]);
        airlines.insert("BAW".to_string(), vec![
            "A319".to_string(), "A320".to_string(), "A321".to_string(), 
            "A20N".to_string(), "A21N".to_string(), "A35K".to_string(), 
            "A388".to_string(), "B772".to_string(), "B788".to_string(), 
            "B789".to_string(), "B78X".to_string()
        ]);
        airlines.insert("EZY".to_string(), vec![
            "A319".to_string(), "A320".to_string(), "A321".to_string(),
            "A20N".to_string(), "A21N".to_string()
        ]);
        airlines.insert("WZZ".to_string(), vec![
            "A320".to_string(), "A321".to_string(), 
            "A20N".to_string(), "A21N".to_string()
        ]);

        let mut airports = HashMap::new();
        airports.insert("EGLL".to_string(), vec![
            "BAW".to_string(), "DLH".to_string(), "EIN".to_string(), 
            "AFR".to_string(), "KLM".to_string(), "UAE".to_string()
        ]);
        airports.insert("EGKK".to_string(), vec![
            "RYR".to_string(), "BAW".to_string(), "EZY".to_string(), 
            "WZZ".to_string(), "DLH".to_string()
        ]);
        airports.insert("EGSS".to_string(), vec![
            "RYR".to_string(), "EZY".to_string(), "WZZ".to_string()
        ]);
        airports.insert("EGGW".to_string(), vec![
            "RYR".to_string(), "EZY".to_string(), "WZZ".to_string()
        ]);
        airports.insert("EGLC".to_string(), vec![
            "BAW".to_string(), "KLM".to_string()
        ]);
        // Add foreign origin airports for transits
        airports.insert("EHAM".to_string(), vec![
            "KLM".to_string(), "BAW".to_string(), "EZY".to_string()
        ]);
        airports.insert("EBBR".to_string(), vec![
            "BAW".to_string(), "DLH".to_string()
        ]);
        airports.insert("EKYT".to_string(), vec![
            "BAW".to_string(), "EZY".to_string()
        ]);
        airports.insert("EGCC".to_string(), vec![
            "BAW".to_string(), "RYR".to_string(), "EZY".to_string()
        ]);
        airports.insert("ESSA".to_string(), vec![
            "BAW".to_string(), "KLM".to_string()
        ]);
        airports.insert("EDDF".to_string(), vec![
            "DLH".to_string(), "BAW".to_string()
        ]);

        Self {
            airlines,
            airports,
        }
    }
}

/// CCAMS squawk ranges
pub fn get_ccams_squawks() -> Vec<u16> {
    let mut squawks = Vec::new();
    
    let ranges = [
        (201, 277), (301, 377), (470, 477), (501, 577),
        (730, 767), (1070, 1077), (1140, 1176), (1410, 1477),
        (2001, 2077), (2150, 2177), (2201, 2277), (2701, 2737),
        (3201, 3277), (3370, 3377), (3401, 3477), (3510, 3537),
        (4215, 4247), (4430, 4477), (4701, 4777), (5013, 5017),
        (5201, 5270), (5401, 5477), (5660, 5664), (5565, 5676),
        (6201, 6257), (6301, 6377), (6460, 6467), (6470, 6477),
        (7014, 7017), (7020, 7027), (7201, 7267), (7270, 7277),
        (7301, 7327), (7501, 7507), (7536, 7537), (7570, 7577),
        (7601, 7617), (7620, 7677), (7701, 7775), (1250, 1257),
        (6001, 6037),
    ];
    
    for (start, end) in ranges {
        squawks.extend(start..=end);
    }
    
    squawks
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_load_profile() -> Result<()> {
        let profile = ProfileConfig::load("profiles/TCE + TCNE.json")?;
        
        println!("Loaded {} departure configs", profile.std_departures.len());
        println!("Loaded {} transit configs", profile.std_transits.len());
        println!("Active airports: {:?}", profile.active_aerodromes);
        println!("Active runways: {:?}", profile.active_runways);
        println!("Master controller: {} on {}", profile.master_controller, profile.master_controller_freq);
        println!("Active controllers: {:?}", profile.active_controllers);
        
        assert!(!profile.std_departures.is_empty());
        assert!(!profile.std_transits.is_empty());
        assert!(!profile.active_aerodromes.is_empty());
        assert!(!profile.active_runways.is_empty());
        
        Ok(())
    }

    #[test]
    fn test_ccams_squawks() {
        let squawks = get_ccams_squawks();
        assert!(!squawks.is_empty());
        println!("Generated {} CCAMS squawks", squawks.len());
    }
}
