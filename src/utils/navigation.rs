/// Navigation utilities and calculations
use std::collections::HashMap;
use std::fs;
use std::path::Path;
use anyhow::{Result, Context};

const EARTH_RADIUS_KM: f64 = 6372.8;
const EARTH_RADIUS_NM: f64 = 3440.065;

pub type FixDatabase = HashMap<String, (f64, f64)>;

pub fn haversine(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let lat1_rad = lat1.to_radians();
    let lat2_rad = lat2.to_radians();

    let a = (dlat / 2.0).sin().powi(2)
        + lat1_rad.cos() * lat2_rad.cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();

    EARTH_RADIUS_KM * c
}

pub fn haversine_nm(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    haversine(lat1, lon1, lat2, lon2) / 1.852
}

pub fn heading_from_to(from_lat: f64, from_lon: f64, to_lat: f64, to_lon: f64) -> i32 {
    let dlon = to_lon - from_lon;
    let y = dlon.to_radians().sin() * to_lat.to_radians().cos();
    let x = from_lat.to_radians().cos() * to_lat.to_radians().sin()
        - from_lat.to_radians().sin()
            * to_lat.to_radians().cos()
            * dlon.to_radians().cos();

    let bearing = y.atan2(x).to_degrees();
    ((bearing + 360.0) % 360.0) as i32
}

pub fn position_bearing_distance(
    lat: f64,
    lon: f64,
    bearing: f64,
    distance_nm: f64,
) -> (f64, f64) {
    let lat_rad = lat.to_radians();
    let lon_rad = lon.to_radians();
    let bearing_rad = bearing.to_radians();

    let angular_distance = distance_nm / EARTH_RADIUS_NM;

    let dest_lat_rad = (lat_rad.sin() * angular_distance.cos()
        + lat_rad.cos() * angular_distance.sin() * bearing_rad.cos())
    .asin();

    let dest_lon_rad = lon_rad
        + (bearing_rad.sin() * angular_distance.sin() * lat_rad.cos())
            .atan2(angular_distance.cos() - lat_rad.sin() * dest_lat_rad.sin());

    (dest_lat_rad.to_degrees(), dest_lon_rad.to_degrees())
}

pub fn delta_position(
    lat: f64,
    tas_knots: f64,
    heading: i32,
    delta_time_seconds: f64,
) -> (f64, f64) {
    let heading_rad = (heading as f64).to_radians();
    let delta_time_hours = delta_time_seconds / 3600.0;

    let delta_lat = (tas_knots * heading_rad.cos() * delta_time_hours) / 60.0;
    let delta_lon =
        (1.0 / lat.to_radians().cos()) * (tas_knots * heading_rad.sin() * delta_time_hours) / 60.0;

    (delta_lat, delta_lon)
}

pub fn shortest_turn_direction(current: i32, target: i32) -> TurnDirection {
    let diff = (target - current + 360) % 360;
    if diff <= 180 {
        TurnDirection::Right
    } else {
        TurnDirection::Left
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TurnDirection {
    Left,
    Right,
}

/// Normalize heading to 0-359 range
pub fn normalize_heading(heading: i32) -> i32 {
    ((heading % 360) + 360) % 360
}

/// Convert sector file coordinates to decimal degrees
/// Format: N050.30.11.880 W003.28.33.640
/// Parts: [N/S][degrees].[minutes].[seconds].[milliseconds]
pub fn sf_coords_to_decimal(lat: &str, lon: &str) -> Result<(f64, f64)> {
    let parse_coord = |coord: &str| -> Result<f64> {
        let parts: Vec<&str> = coord.split('.').collect();
        if parts.len() != 4 {
            anyhow::bail!("Invalid coordinate format: {}", coord);
        }

        let hemisphere = &parts[0][0..1];
        let degrees: f64 = parts[0][1..].parse()?;
        let minutes: f64 = parts[1].parse()?;
        let seconds: f64 = parts[2].parse()?;
        let milliseconds: f64 = parts[3].parse()?;

        let mut decimal = degrees + (minutes / 60.0) + (seconds / 3600.0) + (milliseconds / 3_600_000.0);
        
        if hemisphere == "S" || hemisphere == "W" {
            decimal *= -1.0;
        }

        Ok((decimal * 100_000.0).round() / 100_000.0) // Round to 5 decimal places
    };

    let lat_decimal = parse_coord(lat)?;
    let lon_decimal = parse_coord(lon)?;

    Ok((lat_decimal, lon_decimal))
}

/// Parse a fixes file and return a map of fix name to coordinates
fn parse_fixes_file<P: AsRef<Path>>(path: P) -> Result<FixDatabase> {
    let content = fs::read_to_string(path.as_ref())
        .with_context(|| format!("Failed to read file: {:?}", path.as_ref()))?;

    let mut fixes = HashMap::new();

    for line in content.lines() {
        let line = line.trim();
        
        // Skip comments and empty lines
        if line.is_empty() || line.starts_with(';') {
            continue;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();
        
        if parts.len() >= 3 {
            let fix_name = parts[0].to_string();
            
            // Handle both fix format and VOR/NDB format
            let (lat, lon) = if parts.len() == 3 {
                // Fix format: NAME LAT LON
                (parts[1], parts[2])
            } else {
                // VOR/NDB format: NAME FREQ LAT LON or similar
                // Try to find the coordinate parts
                let lat_idx = parts.iter().position(|&p| p.starts_with('N') || p.starts_with('S'));
                let lon_idx = parts.iter().position(|&p| p.starts_with('E') || p.starts_with('W'));
                
                if let (Some(lat_i), Some(lon_i)) = (lat_idx, lon_idx) {
                    (parts[lat_i], parts[lon_i])
                } else {
                    continue;
                }
            };

            if let Ok(coords) = sf_coords_to_decimal(lat, lon) {
                fixes.insert(fix_name, coords);
            }
        }
    }

    Ok(fixes)
}

/// Parse airport basic data files to get airport reference points
fn parse_airports<P: AsRef<Path>>(airports_dir: P) -> Result<FixDatabase> {
    let mut airports = HashMap::new();
    
    let entries = fs::read_dir(airports_dir.as_ref())
        .with_context(|| format!("Failed to read airports directory: {:?}", airports_dir.as_ref()))?;

    for entry in entries {
        let entry = entry?;
        let path = entry.path();
        
        if path.is_dir() {
            let airport_code = path.file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("")
                .to_string();

            let basic_file = path.join("Basic.txt");
            
            if basic_file.exists() {
                if let Ok(content) = fs::read_to_string(&basic_file) {
                    let lines: Vec<&str> = content.lines().collect();
                    
                    // Line 0: Airport name
                    // Line 1: Coordinates
                    // Line 2: Frequency
                    if lines.len() >= 2 {
                        let coord_parts: Vec<&str> = lines[1].split_whitespace().collect();
                        
                        if coord_parts.len() >= 2 {
                            if let Ok(coords) = sf_coords_to_decimal(coord_parts[0], coord_parts[1]) {
                                airports.insert(airport_code, coords);
                            }
                        }
                    }
                }
            }
        }
    }

    Ok(airports)
}

/// Load all navigation data (fixes, VORs, NDBs, airports)
pub fn load_navigation_data<P: AsRef<Path>>(data_dir: P) -> Result<FixDatabase> {
    let mut all_fixes = HashMap::new();

    let data_path = data_dir.as_ref();
    let navaids_dir = data_path.join("Navaids");
    let airports_dir = data_path.join("Airports");

    // Load primary UK fixes
    if let Ok(fixes) = parse_fixes_file(navaids_dir.join("FIXES_UK.txt")) {
        all_fixes.extend(fixes);
    }

    // Load additional fix files
    let additional_files = vec![
        "FIXES_CICZ.txt",
        "FIXES_Non-UK.txt",
        "FIXES_SIDS-STARS.txt",
        "Fixes_Non-UK/FIXES_Belgium.txt",
        "Fixes_Non-UK/FIXES_Netherlands.txt",
        "Fixes_Non-UK/FIXES_Ireland.txt",
    ];

    for file in additional_files {
        let path = navaids_dir.join(file);
        if path.exists() {
            if let Ok(fixes) = parse_fixes_file(&path) {
                all_fixes.extend(fixes);
            }
        }
    }

    // Load VORs
    for vor_file in &["VOR_UK.txt", "VOR_Non-UK.txt"] {
        let path = navaids_dir.join(vor_file);
        if path.exists() {
            if let Ok(vors) = parse_fixes_file(&path) {
                all_fixes.extend(vors);
            }
        }
    }

    // Load NDBs
    let ndb_path = navaids_dir.join("NDB_All.txt");
    if ndb_path.exists() {
        if let Ok(ndbs) = parse_fixes_file(&ndb_path) {
            all_fixes.extend(ndbs);
        }
    }

    // Load airports
    if airports_dir.exists() {
        if let Ok(airports) = parse_airports(&airports_dir) {
            all_fixes.extend(airports);
        }
        
        // Load SID waypoints from airport folders
        if let Ok(sid_fixes) = parse_sid_waypoints(&airports_dir) {
            all_fixes.extend(sid_fixes);
        }
    }

    Ok(all_fixes)
}

/// Parse SID waypoints from airport folders
fn parse_sid_waypoints<P: AsRef<Path>>(airports_dir: P) -> Result<FixDatabase> {
    let mut waypoints = HashMap::new();
    
    let entries = fs::read_dir(airports_dir.as_ref())
        .with_context(|| format!("Failed to read airports directory: {:?}", airports_dir.as_ref()))?;

    for entry in entries {
        let entry = entry?;
        let path = entry.path();
        
        if path.is_dir() {
            // Load the Fixes.txt file from each airport folder
            let fixes_file = path.join("Fixes.txt");
            
            if fixes_file.exists() {
                if let Ok(fixes) = parse_fixes_file(&fixes_file) {
                    waypoints.extend(fixes);
                }
            }
        }
    }

    Ok(waypoints)
}

/// Get coordinates for a fix/navaid/airport
pub fn get_fix_coords(fixes: &FixDatabase, fix_name: &str) -> Option<(f64, f64)> {
    fixes.get(fix_name).copied()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haversine() {
        let dist = haversine(51.5074, -0.1278, 48.8566, 2.3522);
        assert!((dist - 340.0).abs() < 10.0);
    }

    #[test]
    fn test_heading() {
        let hdg = heading_from_to(50.0, 0.0, 51.0, 0.0);
        assert!((hdg - 0).abs() < 5);

        let hdg = heading_from_to(50.0, 0.0, 50.0, 1.0);
        assert!((hdg - 90).abs() < 5);
    }

    #[test]
    fn test_shortest_turn() {
        assert_eq!(shortest_turn_direction(10, 20), TurnDirection::Right);
        assert_eq!(shortest_turn_direction(350, 10), TurnDirection::Right);
        assert_eq!(shortest_turn_direction(20, 350), TurnDirection::Left);
    }

    #[test]
    fn test_sf_coords_conversion() {
        // Test ABBEW N050.30.11.880 W003.28.33.640
        let (lat, lon) = sf_coords_to_decimal("N050.30.11.880", "W003.28.33.640").unwrap();
        assert!((lat - 50.50330).abs() < 0.001);
        assert!((lon - (-3.47601)).abs() < 0.001);
    }

    #[test]
    fn test_southern_western_hemisphere() {
        let (lat, lon) = sf_coords_to_decimal("S010.00.00.000", "W020.00.00.000").unwrap();
        assert_eq!(lat, -10.0);
        assert_eq!(lon, -20.0);
    }
}
