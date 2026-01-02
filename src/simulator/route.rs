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
        // Basic parsing - split by spaces and extract fix names
        let parts: Vec<&str> = self.route_string.split_whitespace().collect();
        
        for part in parts {
            // Skip airway names (typically start with letters and numbers)
            // Keep fix names (typically all caps)
            if part.chars().all(|c| c.is_uppercase() || c.is_numeric()) && part.len() >= 3 {
                self.fixes.push(part.to_string());
            }
        }
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
        write!(f, "{}", self.route_string)
    }
}
