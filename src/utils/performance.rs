use std::collections::HashMap;
use std::fs;
use std::path::Path;
use anyhow::{Result, Context};

/// Performance data for a specific altitude band
#[derive(Debug, Clone)]
pub struct PerformanceLine {
    pub flight_level: u32,        // Flight level (e.g., 030 = 3000ft)
    pub climb_speed: u32,          // Climb speed in knots (0 if Mach only)
    pub cruise_speed: u32,         // Cruise speed in knots
    pub descent_speed: u32,        // Descent speed in knots
    pub climb_mach: f64,           // Climb Mach (0 if knots only)
    pub cruise_mach: f64,          // Cruise Mach
    pub descent_mach: f64,         // Descent Mach
    pub rate_of_climb: i32,        // Rate of climb in ft/min
    pub rate_of_descent: i32,      // Rate of descent in ft/min (positive value)
}

/// Complete performance profile for an aircraft type
#[derive(Debug, Clone)]
pub struct AircraftPerformance {
    pub aircraft_type: String,
    pub performance_lines: Vec<PerformanceLine>,
}

impl AircraftPerformance {
    /// Get performance data for a specific altitude
    /// Returns the appropriate performance line based on altitude
    pub fn get_performance_at_altitude(&self, altitude_ft: f64) -> Option<&PerformanceLine> {
        let altitude_fl = (altitude_ft / 100.0) as u32;
        
        // Find the appropriate performance line for this altitude
        // Use the highest FL that is <= our current altitude
        self.performance_lines
            .iter()
            .filter(|line| line.flight_level <= altitude_fl)
            .max_by_key(|line| line.flight_level)
    }

    /// Get the rate of climb for a specific altitude
    pub fn get_rate_of_climb(&self, altitude_ft: f64) -> i32 {
        self.get_performance_at_altitude(altitude_ft)
            .map(|perf| perf.rate_of_climb)
            .unwrap_or(2000) // Default fallback
    }

    /// Get the rate of descent for a specific altitude
    pub fn get_rate_of_descent(&self, altitude_ft: f64) -> i32 {
        self.get_performance_at_altitude(altitude_ft)
            .map(|perf| -perf.rate_of_descent) // Negative for descent
            .unwrap_or(-2000) // Default fallback
    }

    /// Get appropriate speed for climbing at altitude
    pub fn get_climb_speed(&self, altitude_ft: f64) -> u32 {
        self.get_performance_at_altitude(altitude_ft)
            .map(|perf| {
                if perf.climb_speed > 0 {
                    perf.climb_speed
                } else {
                    perf.cruise_speed // Fallback to cruise if no climb speed
                }
            })
            .unwrap_or(250)
    }

    /// Get appropriate speed for descent at altitude
    pub fn get_descent_speed(&self, altitude_ft: f64) -> u32 {
        self.get_performance_at_altitude(altitude_ft)
            .map(|perf| {
                if perf.descent_speed > 0 {
                    perf.descent_speed
                } else {
                    perf.cruise_speed
                }
            })
            .unwrap_or(250)
    }
}

pub type PerformanceDatabase = HashMap<String, AircraftPerformance>;

/// Parse a PERFLINE entry
fn parse_perf_line(line: &str) -> Result<PerformanceLine> {
    let parts: Vec<&str> = line.split(':').collect();
    
    if parts.len() != 9 || parts[0] != "PERFLINE" {
        anyhow::bail!("Invalid PERFLINE format: {}", line);
    }

    Ok(PerformanceLine {
        flight_level: parts[1].parse()?,
        climb_speed: parts[2].parse()?,
        cruise_speed: parts[3].parse()?,
        descent_speed: parts[4].parse()?,
        climb_mach: if parts[5] == "0" { 0.0 } else { parts[5].parse::<f64>()? / 100.0 },
        cruise_mach: if parts[6] == "0" { 0.0 } else { parts[6].parse::<f64>()? / 100.0 },
        descent_mach: if parts[7] == "0" { 0.0 } else { parts[7].parse::<f64>()? / 100.0 },
        rate_of_climb: parts[8].split(':').next().unwrap_or(parts[8]).parse()?,
        rate_of_descent: parts[8].split(':').last().unwrap_or(parts[8]).parse()?,
    })
}

/// Load aircraft performance data from file
pub fn load_performance_data<P: AsRef<Path>>(path: P) -> Result<PerformanceDatabase> {
    let content = fs::read_to_string(path.as_ref())
        .with_context(|| format!("Failed to read performance file: {:?}", path.as_ref()))?;

    let mut database = HashMap::new();
    let mut current_aircraft: Option<String> = None;
    let mut current_lines: Vec<PerformanceLine> = Vec::new();

    for line in content.lines() {
        let line = line.trim();
        
        // Skip comments and empty lines
        if line.is_empty() || line.starts_with('-') {
            continue;
        }

        if line.starts_with("PERFAC:") {
            // Save previous aircraft if exists
            if let Some(aircraft_type) = current_aircraft.take() {
                if !current_lines.is_empty() {
                    database.insert(
                        aircraft_type.clone(),
                        AircraftPerformance {
                            aircraft_type,
                            performance_lines: current_lines.clone(),
                        },
                    );
                    current_lines.clear();
                }
            }

            // Start new aircraft
            current_aircraft = Some(line[7..].to_string());
        } else if line.starts_with("PERFLINE:") {
            if let Ok(perf_line) = parse_perf_line(line) {
                current_lines.push(perf_line);
            }
        }
    }

    // Don't forget the last aircraft
    if let Some(aircraft_type) = current_aircraft {
        if !current_lines.is_empty() {
            database.insert(
                aircraft_type.clone(),
                AircraftPerformance {
                    aircraft_type,
                    performance_lines: current_lines,
                },
            );
        }
    }

    Ok(database)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_perf_line() {
        let line = "PERFLINE:030:190:230:210:0:0:0:2800:900";
        let perf = parse_perf_line(line).unwrap();
        
        assert_eq!(perf.flight_level, 30);
        assert_eq!(perf.climb_speed, 190);
        assert_eq!(perf.cruise_speed, 230);
        assert_eq!(perf.rate_of_climb, 2800);
        assert_eq!(perf.rate_of_descent, 900);
    }

    #[test]
    fn test_get_performance_at_altitude() {
        let perf = AircraftPerformance {
            aircraft_type: "TEST".to_string(),
            performance_lines: vec![
                PerformanceLine {
                    flight_level: 30,
                    climb_speed: 190,
                    cruise_speed: 230,
                    descent_speed: 210,
                    climb_mach: 0.0,
                    cruise_mach: 0.0,
                    descent_mach: 0.0,
                    rate_of_climb: 2800,
                    rate_of_descent: 900,
                },
                PerformanceLine {
                    flight_level: 100,
                    climb_speed: 250,
                    cruise_speed: 250,
                    descent_speed: 250,
                    climb_mach: 0.0,
                    cruise_mach: 0.0,
                    descent_mach: 0.0,
                    rate_of_climb: 2600,
                    rate_of_descent: 1500,
                },
            ],
        };

        // At 5000ft, should use FL030 data
        let p = perf.get_performance_at_altitude(5000.0).unwrap();
        assert_eq!(p.rate_of_climb, 2800);

        // At 12000ft, should use FL100 data
        let p = perf.get_performance_at_altitude(12000.0).unwrap();
        assert_eq!(p.rate_of_climb, 2600);
    }
}
