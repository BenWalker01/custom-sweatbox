/// Navigation utilities and calculations
use std::f64::consts::PI;

const EARTH_RADIUS_KM: f64 = 6372.8;
const EARTH_RADIUS_NM: f64 = 3440.065;

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
}
