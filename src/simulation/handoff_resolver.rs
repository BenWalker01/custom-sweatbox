use anyhow::Result;
use std::collections::{HashMap, HashSet};
use std::fs;
use std::path::Path;

use crate::aircraft::Aircraft;
use crate::utils::navigation::{haversine_nm, sf_coords_to_decimal, FixDatabase};

#[derive(Debug, Clone)]
pub struct OwnershipDecision {
    pub owner_callsign: String,
    pub sector_name: String,
}

#[derive(Debug, Clone)]
pub struct OwnershipResolver {
    airport_rules: Vec<AirportRuleSet>,
    callsigns_by_code: HashMap<String, Vec<String>>,
    known_callsigns: HashSet<String>,
    active_runways: HashMap<String, String>,
}

#[derive(Debug, Clone)]
struct AirportRuleSet {
    sectors: Vec<SectorRule>,
}

#[derive(Debug, Clone)]
struct SectorRule {
    name: String,
    min_alt_ft: i32,
    max_alt_ft: i32,
    owner_priority: Vec<String>,
    borders: Vec<String>,
    dep_apts: Vec<String>,
    arr_apts: Vec<String>,
    active_runway_filters: HashMap<String, HashSet<String>>,
    geometry: Option<Geometry>,
}

#[derive(Debug, Clone)]
enum Geometry {
    Circle {
        center_lat: f64,
        center_lon: f64,
        radius_nm: f64,
    },
    Polygon {
        points: Vec<(f64, f64)>,
    },
}

#[derive(Default)]
struct SectorGeometryIndex {
    circles: HashMap<String, (f64, f64, f64)>,
    lines: HashMap<String, Vec<(f64, f64)>>,
}

impl OwnershipResolver {
    pub fn from_scenario_data(
        active_aerodromes: &[String],
        active_runways: &HashMap<String, String>,
        active_controllers: &[String],
        master_controller: &str,
        other_controllers: &[(String, String)],
        inactive_sectors: &[String],
        nav_db: &FixDatabase,
    ) -> Result<Self> {
        let mut controller_order = Vec::new();
        let mut seen_callsigns = HashSet::new();
        for callsign in active_controllers {
            let normalized = callsign.trim().to_uppercase();
            if !normalized.is_empty() && seen_callsigns.insert(normalized.clone()) {
                controller_order.push(normalized);
            }
        }

        let master = master_controller.trim().to_uppercase();
        if !master.is_empty() && seen_callsigns.insert(master.clone()) {
            controller_order.push(master);
        }

        for (callsign, _) in other_controllers {
            let normalized = callsign.trim().to_uppercase();
            if !normalized.is_empty() && seen_callsigns.insert(normalized.clone()) {
                controller_order.push(normalized);
            }
        }

        let inactive_callsigns: HashSet<String> = inactive_sectors
            .iter()
            .map(|s| s.trim().to_uppercase())
            .filter(|s| !s.is_empty())
            .collect();

        let callsign_to_code = Self::build_callsign_code_map(&controller_order)?;
        let mut callsigns_by_code: HashMap<String, Vec<String>> = HashMap::new();

        for callsign in &controller_order {
            if inactive_callsigns.contains(callsign) {
                continue;
            }

            if let Some(code) = callsign_to_code.get(callsign) {
                callsigns_by_code
                    .entry(code.clone())
                    .or_default()
                    .push(callsign.clone());
            }
        }

        let mut airport_rules = Vec::new();
        for airport in active_aerodromes {
            let airport_code = airport.trim().to_uppercase();
            if airport_code.is_empty() {
                continue;
            }

            let ownership_path = format!("data/Airports/{}/Ownership.txt", airport_code);
            let sector_path = format!("data/Airports/{}/Sector.txt", airport_code);

            if !Path::new(&ownership_path).exists() || !Path::new(&sector_path).exists() {
                continue;
            }

            let geometry = Self::parse_sector_geometry(&sector_path, nav_db)?;
            let mut rules = Self::parse_ownership_rules(&ownership_path)?;
            for rule in &mut rules {
                rule.geometry = Self::resolve_rule_geometry(rule, &geometry);
            }
            rules.retain(|r| r.geometry.is_some() && !r.owner_priority.is_empty());

            airport_rules.push(AirportRuleSet { sectors: rules });
        }

        Ok(Self {
            airport_rules,
            callsigns_by_code,
            known_callsigns: controller_order.into_iter().collect(),
            active_runways: active_runways
                .iter()
                .map(|(k, v)| (k.trim().to_uppercase(), v.trim().to_uppercase()))
                .collect(),
        })
    }

    pub fn resolve_owner_for_aircraft(&self, aircraft: &Aircraft) -> Option<OwnershipDecision> {
        self.resolve_owner(
            aircraft.latitude,
            aircraft.longitude,
            aircraft.altitude,
            &aircraft.flight_plan.departure,
            &aircraft.flight_plan.arrival,
        )
    }

    pub fn resolve_owner(
        &self,
        lat: f64,
        lon: f64,
        altitude_ft: i32,
        departure: &str,
        arrival: &str,
    ) -> Option<OwnershipDecision> {
        let departure = departure.trim().to_uppercase();
        let arrival = arrival.trim().to_uppercase();

        for airport_rules in &self.airport_rules {
            for rule in &airport_rules.sectors {
                if !rule.matches_flight_context(
                    altitude_ft,
                    &departure,
                    &arrival,
                    &self.active_runways,
                ) {
                    continue;
                }

                if !rule.contains_point(lat, lon) {
                    continue;
                }

                if let Some(owner_callsign) = self.select_owner_callsign(&rule.owner_priority) {
                    return Some(OwnershipDecision {
                        owner_callsign,
                        sector_name: rule.name.clone(),
                    });
                }
            }
        }

        None
    }

    fn select_owner_callsign(&self, owners: &[String]) -> Option<String> {
        for owner in owners {
            let normalized = owner.trim().to_uppercase();
            if normalized.is_empty() {
                continue;
            }

            if let Some(callsigns) = self.callsigns_by_code.get(&normalized) {
                if let Some(callsign) = callsigns.first() {
                    return Some(callsign.clone());
                }
            } else if self.known_callsigns.contains(&normalized) {
                return Some(normalized);
            }
        }

        None
    }

    fn build_callsign_code_map(controller_order: &[String]) -> Result<HashMap<String, String>> {
        let wanted: HashSet<&str> = controller_order.iter().map(|s| s.as_str()).collect();
        let mut result = HashMap::new();

        let area_positions = Path::new("data/UK-Sector-File/Area Positions/1 UK Permanent.txt");
        if area_positions.exists() {
            Self::parse_positions_file(area_positions, &wanted, &mut result)?;
        }

        let airports_root = Path::new("data/Airports");
        if airports_root.exists() {
            for entry in fs::read_dir(airports_root)? {
                let entry = entry?;
                let path = entry.path();
                if !path.is_dir() {
                    continue;
                }

                let positions_path = path.join("Positions.txt");
                if positions_path.exists() {
                    Self::parse_positions_file(&positions_path, &wanted, &mut result)?;
                }
            }
        }

        Ok(result)
    }

    fn parse_positions_file(
        path: &Path,
        wanted_callsigns: &HashSet<&str>,
        output: &mut HashMap<String, String>,
    ) -> Result<()> {
        let content = fs::read_to_string(path)?;
        for line in content.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with(';') {
                continue;
            }

            let parts: Vec<&str> = trimmed.split(':').collect();
            if parts.len() < 4 {
                continue;
            }

            let callsign = parts[0].trim().to_uppercase();
            if !wanted_callsigns.contains(callsign.as_str()) {
                continue;
            }

            let code = parts[3].trim().to_uppercase();
            if code.is_empty() {
                continue;
            }

            output.entry(callsign).or_insert(code);
        }

        Ok(())
    }

    fn parse_sector_geometry(path: &str, nav_db: &FixDatabase) -> Result<SectorGeometryIndex> {
        let content = fs::read_to_string(path)?;
        let mut index = SectorGeometryIndex::default();
        let mut current_line_name: Option<String> = None;

        for line in content.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with(';') {
                continue;
            }

            if let Some(rest) = trimmed.strip_prefix("CIRCLE_SECTORLINE:") {
                current_line_name = None;
                let parts: Vec<&str> = rest.split(':').collect();
                if parts.len() < 3 {
                    continue;
                }

                let name = parts[0].trim().to_uppercase();
                let center_ref = parts[1].trim().to_uppercase();
                let radius_nm = parts[2].trim().parse::<f64>().unwrap_or(0.0);
                if radius_nm <= 0.0 {
                    continue;
                }

                if let Some((lat, lon)) = nav_db.get(&center_ref) {
                    index.circles.insert(name, (*lat, *lon, radius_nm));
                }
                continue;
            }

            if let Some(rest) = trimmed.strip_prefix("SECTORLINE:") {
                current_line_name = Some(rest.trim().to_uppercase());
                continue;
            }

            if let Some(rest) = trimmed.strip_prefix("COORD:") {
                let Some(line_name) = current_line_name.as_ref() else {
                    continue;
                };

                let coord_data = rest.split(';').next().unwrap_or(rest).trim();
                let parts: Vec<&str> = coord_data.split(':').collect();
                if parts.len() < 2 {
                    continue;
                }

                let lat_raw = parts[0].trim();
                let lon_raw = parts[1].trim();
                if let Ok((lat, lon)) = sf_coords_to_decimal(lat_raw, lon_raw) {
                    index
                        .lines
                        .entry(line_name.clone())
                        .or_default()
                        .push((lat, lon));
                }
            }
        }

        Ok(index)
    }

    fn parse_ownership_rules(path: &str) -> Result<Vec<SectorRule>> {
        let content = fs::read_to_string(path)?;
        let mut rules = Vec::new();
        let mut current: Option<SectorRule> = None;

        for line in content.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with(';') {
                continue;
            }

            if let Some(rest) = trimmed.strip_prefix("SECTOR:") {
                if let Some(rule) = current.take() {
                    rules.push(rule);
                }

                let parts: Vec<&str> = rest.split(':').collect();
                if parts.len() < 3 {
                    continue;
                }

                let name = parts[0].trim().to_uppercase();
                let min_alt_ft = parts[1].trim().parse::<i32>().unwrap_or(0);
                let max_alt_ft = parts[2].trim().parse::<i32>().unwrap_or(0);

                current = Some(SectorRule {
                    name,
                    min_alt_ft,
                    max_alt_ft,
                    owner_priority: Vec::new(),
                    borders: Vec::new(),
                    dep_apts: Vec::new(),
                    arr_apts: Vec::new(),
                    active_runway_filters: HashMap::new(),
                    geometry: None,
                });
                continue;
            }

            let Some(rule) = current.as_mut() else {
                continue;
            };

            if let Some(rest) = trimmed.strip_prefix("OWNER:") {
                rule.owner_priority = rest
                    .split(':')
                    .map(|s| s.trim().to_uppercase())
                    .filter(|s| !s.is_empty())
                    .collect();
                continue;
            }

            if let Some(rest) = trimmed.strip_prefix("BORDER:") {
                rule.borders = rest
                    .split(':')
                    .map(|s| s.trim().to_uppercase())
                    .filter(|s| !s.is_empty())
                    .collect();
                continue;
            }

            if let Some(rest) = trimmed.strip_prefix("DEPAPT:") {
                rule.dep_apts = rest
                    .split(':')
                    .map(|s| s.trim().to_uppercase())
                    .filter(|s| !s.is_empty())
                    .collect();
                continue;
            }

            if let Some(rest) = trimmed.strip_prefix("ARRAPT:") {
                rule.arr_apts = rest
                    .split(':')
                    .map(|s| s.trim().to_uppercase())
                    .filter(|s| !s.is_empty())
                    .collect();
                continue;
            }

            if let Some(rest) = trimmed.strip_prefix("ACTIVE:") {
                let parts: Vec<&str> = rest.split(':').collect();
                if parts.len() >= 2 {
                    let airport = parts[0].trim().to_uppercase();
                    let runway = parts[1].trim().to_uppercase();
                    if !airport.is_empty() && !runway.is_empty() {
                        rule.active_runway_filters
                            .entry(airport)
                            .or_default()
                            .insert(runway);
                    }
                }
            }
        }

        if let Some(rule) = current.take() {
            rules.push(rule);
        }

        Ok(rules)
    }

    fn resolve_rule_geometry(
        rule: &SectorRule,
        geometry: &SectorGeometryIndex,
    ) -> Option<Geometry> {
        if !rule.borders.is_empty() {
            if rule.borders.len() == 1 {
                let border = &rule.borders[0];
                if let Some((lat, lon, radius_nm)) = geometry.circles.get(border) {
                    return Some(Geometry::Circle {
                        center_lat: *lat,
                        center_lon: *lon,
                        radius_nm: *radius_nm,
                    });
                }
            }

            let mut polylines = Vec::new();
            for border in &rule.borders {
                if let Some(points) = geometry.lines.get(border) {
                    if !points.is_empty() {
                        polylines.push(points.clone());
                    }
                }
            }

            if let Some(points) = Self::stitch_polylines(polylines) {
                return Some(Geometry::Polygon { points });
            }
        } else {
            if let Some((lat, lon, radius_nm)) = geometry.circles.get(&rule.name) {
                return Some(Geometry::Circle {
                    center_lat: *lat,
                    center_lon: *lon,
                    radius_nm: *radius_nm,
                });
            }

            if let Some(points) = geometry.lines.get(&rule.name) {
                if points.len() >= 3 {
                    let mut closed = points.clone();
                    if closed.first() != closed.last() {
                        closed.push(closed[0]);
                    }
                    return Some(Geometry::Polygon { points: closed });
                }
            }
        }

        None
    }

    fn stitch_polylines(mut polylines: Vec<Vec<(f64, f64)>>) -> Option<Vec<(f64, f64)>> {
        if polylines.is_empty() {
            return None;
        }

        let mut path = polylines.remove(0);
        while let Some(mut segment) = polylines.pop() {
            if segment.is_empty() {
                continue;
            }

            let Some(path_end) = path.last().copied() else {
                path = segment;
                continue;
            };
            let path_start = path[0];
            let seg_start = segment[0];
            let seg_end = *segment.last().unwrap_or(&seg_start);

            let end_start = haversine_nm(path_end.0, path_end.1, seg_start.0, seg_start.1);
            let end_end = haversine_nm(path_end.0, path_end.1, seg_end.0, seg_end.1);
            let start_start = haversine_nm(path_start.0, path_start.1, seg_start.0, seg_start.1);
            let start_end = haversine_nm(path_start.0, path_start.1, seg_end.0, seg_end.1);

            if end_start <= end_end && end_start <= start_start && end_start <= start_end {
                path.extend(segment.into_iter().skip(1));
            } else if end_end <= start_start && end_end <= start_end {
                segment.reverse();
                path.extend(segment.into_iter().skip(1));
            } else if start_end <= start_start {
                let mut prefix = segment;
                prefix.pop();
                prefix.extend(path);
                path = prefix;
            } else {
                segment.reverse();
                let mut prefix = segment;
                prefix.pop();
                prefix.extend(path);
                path = prefix;
            }
        }

        if path.len() < 3 {
            return None;
        }

        if path.first() != path.last() {
            path.push(path[0]);
        }

        Some(path)
    }
}

impl SectorRule {
    fn matches_flight_context(
        &self,
        altitude_ft: i32,
        departure: &str,
        arrival: &str,
        active_runways: &HashMap<String, String>,
    ) -> bool {
        if self.max_alt_ft == 0 && self.min_alt_ft == 0 {
            if altitude_ft > 200 {
                return false;
            }
        } else if self.max_alt_ft > 0 {
            if altitude_ft < self.min_alt_ft || altitude_ft > self.max_alt_ft {
                return false;
            }
        } else if altitude_ft < self.min_alt_ft {
            return false;
        }

        if !self.dep_apts.is_empty()
            && !self
                .dep_apts
                .iter()
                .any(|dep| dep.eq_ignore_ascii_case(departure))
        {
            return false;
        }

        if !self.arr_apts.is_empty()
            && !self
                .arr_apts
                .iter()
                .any(|arr_apt| arr_apt.eq_ignore_ascii_case(arrival))
        {
            return false;
        }

        for (airport, allowed_runways) in &self.active_runway_filters {
            let Some(active) = active_runways.get(airport) else {
                return false;
            };
            if !allowed_runways.contains(&active.to_uppercase()) {
                return false;
            }
        }

        true
    }

    fn contains_point(&self, lat: f64, lon: f64) -> bool {
        let Some(geometry) = self.geometry.as_ref() else {
            return false;
        };

        match geometry {
            Geometry::Circle {
                center_lat,
                center_lon,
                radius_nm,
            } => haversine_nm(lat, lon, *center_lat, *center_lon) <= *radius_nm,
            Geometry::Polygon { points } => point_in_polygon(lat, lon, points),
        }
    }
}

fn point_in_polygon(lat: f64, lon: f64, polygon: &[(f64, f64)]) -> bool {
    if polygon.len() < 3 {
        return false;
    }

    let mut inside = false;
    let mut j = polygon.len() - 1;

    for i in 0..polygon.len() {
        let (lat_i, lon_i) = polygon[i];
        let (lat_j, lon_j) = polygon[j];

        let delta_lon = lon_j - lon_i;
        if delta_lon.abs() < 1e-9 {
            j = i;
            continue;
        }

        let intersects = ((lon_i > lon) != (lon_j > lon))
            && (lat < (lat_j - lat_i) * (lon - lon_i) / delta_lon + lat_i);

        if intersects {
            inside = !inside;
        }

        j = i;
    }

    inside
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_in_polygon() {
        let poly = vec![
            (51.0, 0.0),
            (51.0, 1.0),
            (52.0, 1.0),
            (52.0, 0.0),
            (51.0, 0.0),
        ];

        assert!(point_in_polygon(51.5, 0.5, &poly));
        assert!(!point_in_polygon(53.0, 0.5, &poly));
    }

    #[test]
    fn test_resolve_owner_priority() {
        let rule = SectorRule {
            name: "TEST".to_string(),
            min_alt_ft: 0,
            max_alt_ft: 10000,
            owner_priority: vec!["AAA".to_string(), "BBB".to_string()],
            borders: vec![],
            dep_apts: vec![],
            arr_apts: vec![],
            active_runway_filters: HashMap::new(),
            geometry: Some(Geometry::Circle {
                center_lat: 51.0,
                center_lon: 0.0,
                radius_nm: 100.0,
            }),
        };

        let resolver = OwnershipResolver {
            airport_rules: vec![AirportRuleSet {
                sectors: vec![rule],
            }],
            callsigns_by_code: HashMap::from([
                ("AAA".to_string(), vec!["FIRST_CTRL".to_string()]),
                ("BBB".to_string(), vec!["SECOND_CTRL".to_string()]),
            ]),
            known_callsigns: HashSet::new(),
            active_runways: HashMap::new(),
        };

        let decision = resolver
            .resolve_owner(51.0, 0.0, 5000, "EGSS", "EHAM")
            .expect("expected owner");
        assert_eq!(decision.owner_callsign, "FIRST_CTRL");
        assert_eq!(decision.sector_name, "TEST");
    }
}
