use crate::utils::procedures::{load_procedures, ProcedureDatabase};

#[derive(Debug, Clone)]
pub struct Route {
    pub route_string: String,
    pub fixes: Vec<String>,
    pub dep_ad: String,
    pub arr_ad: Option<String>,
    pub initial: bool,
    pub star_intermediate_route: Option<String>,
}

impl Route {
    pub fn new(route: String, dep_ad: String, arr_ad: Option<String>) -> Self {
        let mut route_obj = Self {
            route_string: route.clone(),
            fixes: Vec::new(),
            dep_ad: dep_ad.clone(),
            arr_ad: arr_ad.clone(),
            initial: true,
            star_intermediate_route: None,
        };

        route_obj.initialize_fixes_from_route();
        route_obj
    }

    fn initialize_fixes_from_route(&mut self) {
        let mut fix_airways: Vec<String> = self.route_string
            .split_whitespace()
            .map(|s| s.to_string())
            .collect();

        // Handle SID parsing (e.g., "BPK5K/09L")
        if self.dep_ad.starts_with("EG") && !fix_airways.is_empty() {
            if let Some(sid_fixes) = self.expand_sid(&fix_airways[0]) {
                fix_airways[0] = sid_fixes;
            }
        }

        // Handle STAR parsing (e.g., "ALESO1H/27R")
        if let Some(ref arr_ad) = self.arr_ad {
            if arr_ad.starts_with("EG") && !fix_airways.is_empty() {
                let last_idx = fix_airways.len() - 1;
                if let Some(star_fixes) = self.expand_star(&fix_airways[last_idx]) {
                    fix_airways[last_idx] = star_fixes;
                }
            }
        }

        let mut prev_wpt: Option<String> = None;
        let mut prev_route: Option<String> = None;

        for fix in fix_airways {
            // Remove level/speed restrictions (e.g., "POL/N0272F180" -> "POL")
            let fix = if let Some(idx) = fix.find('/') {
                fix[..idx].to_string()
            } else {
                fix
            };

            // Check if this is a fix (waypoint) we can navigate to
            // In full implementation, would check against FIXES database
            let is_fix = fix.chars().all(|c| c.is_uppercase()) && fix.len() >= 3 && fix.len() <= 5;

            if is_fix {
                // If we have a previous waypoint and airway, expand the airway
                if let (Some(prev_wp), Some(route_name)) = (prev_wpt.as_ref(), prev_route.as_ref()) {
                    // TODO: Look up airway in ATS_DATA and add intermediate fixes
                    // For now, just add the fixes directly
                    self.expand_airway(prev_wp, &fix, route_name);
                    prev_wpt = None;
                    prev_route = None;
                }

                prev_wpt = Some(fix.clone());
                self.fixes.push(fix);
            } else {
                // This might be an airway name
                // Airway names typically: L9, UL9, M197, UN57, etc.
                if Self::looks_like_airway(&fix) {
                    prev_route = Some(fix);
                } else {
                    prev_route = None;
                }
            }
        }
    }

    fn looks_like_airway(s: &str) -> bool {
        // Airways typically start with U, L, M, N, P, Q, T, Y, Z
        // followed by letters and numbers
        if s.is_empty() {
            return false;
        }
        
        let first_char = s.chars().next().unwrap();
        matches!(first_char, 'U' | 'L' | 'M' | 'N' | 'P' | 'Q' | 'R' | 'T' | 'Y' | 'Z')
            && s.len() >= 2
    }

    /// Expand SID if route starts with "SIDNAME/RUNWAY" format
    /// Returns the expanded fix string if SID found, None otherwise
    fn expand_sid(&self, first_item: &str) -> Option<String> {
        // Check for SID format: SIDNAME/RUNWAY (e.g., "BPK5K/09L")
        if let Some(slash_idx) = first_item.find('/') {
            let sid_name = &first_item[..slash_idx];
            let runway = &first_item[slash_idx + 1..];

            // Load SID data for departure airport
            if let Ok((sids, _)) = load_procedures("data", &self.dep_ad) {
                if let Some(sid_runways) = sids.get(sid_name) {
                    if let Some(fixes) = sid_runways.get(runway) {
                        println!("Expanded SID {} for runway {} at {}: {}", 
                                 sid_name, runway, self.dep_ad, fixes);
                        return Some(fixes.clone());
                    }
                }
            }
        }
        None
    }

    /// Expand STAR if route ends with "STARNAME/RUNWAY" format
    /// Returns the expanded fix string if STAR found, None otherwise
    fn expand_star(&self, last_item: &str) -> Option<String> {
        // Check for STAR format: STARNAME/RUNWAY (e.g., "ALESO1H/27R")
        if let Some(slash_idx) = last_item.find('/') {
            let star_name = &last_item[..slash_idx];
            let runway = &last_item[slash_idx + 1..];

            if let Some(ref arr_ad) = self.arr_ad {
                // Load STAR data for arrival airport
                if let Ok((_, stars)) = load_procedures("data", arr_ad) {
                    if let Some(star_runways) = stars.get(star_name) {
                        if let Some(fixes) = star_runways.get(runway) {
                            println!("Expanded STAR {} for runway {} at {}: {}", 
                                     star_name, runway, arr_ad, fixes);
                            return Some(fixes.clone());
                        }
                    }
                }
            }
        }
        None
    }

    fn expand_airway(&mut self, from_fix: &str, to_fix: &str, _airway: &str) {
        // TODO: Load ATS route data and expand intermediate fixes
        // For now, this is a placeholder that just connects the two fixes
        // In full implementation:
        // 1. Look up airway in ATS_DATA
        // 2. Find index of from_fix and to_fix
        // 3. Iterate through fixes in correct direction
        // 4. Add all intermediate fixes to self.fixes
        
        // Placeholder: just add the destination fix
        // (from_fix is already added before calling this)
    }

    pub fn remove_first_fix(&mut self) {
        if !self.fixes.is_empty() {
            self.fixes.remove(0);
        }
    }

    pub fn duplicate(&self) -> Self {
        Self {
            route_string: self.route_string.clone(),
            fixes: self.fixes.clone(),
            dep_ad: self.dep_ad.clone(),
            arr_ad: self.arr_ad.clone(),
            initial: self.initial,
            star_intermediate_route: self.star_intermediate_route.clone(),
        }
    }
}

impl std::fmt::Display for Route {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(ref star) = self.star_intermediate_route {
            write!(f, "{}", star)
        } else {
            write!(f, "{}", self.route_string)
        }
    }
}
