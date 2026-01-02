use super::route::Route;

#[derive(Debug, Clone)]
pub struct FlightPlan {
    pub flight_rules: String,
    pub aircraft_type: String,
    pub enroute_speed: u32,
    pub departure: String,
    pub off_block_time: u32,
    pub enroute_time: u32,
    pub cruise_altitude: u32,
    pub destination: String,
    pub route: Route,
}

impl FlightPlan {
    pub fn new(
        flight_rules: String,
        aircraft_type: String,
        enroute_speed: u32,
        departure: String,
        off_block_time: u32,
        enroute_time: u32,
        cruise_altitude: u32,
        destination: String,
        route: Route,
    ) -> Self {
        Self {
            flight_rules,
            aircraft_type,
            enroute_speed,
            departure,
            off_block_time,
            enroute_time,
            cruise_altitude,
            destination,
            route,
        }
    }

    pub fn duplicate(&self) -> Self {
        Self {
            flight_rules: self.flight_rules.clone(),
            aircraft_type: self.aircraft_type.clone(),
            enroute_speed: self.enroute_speed,
            departure: self.departure.clone(),
            off_block_time: self.off_block_time,
            enroute_time: self.enroute_time,
            cruise_altitude: self.cruise_altitude,
            destination: self.destination.clone(),
            route: self.route.duplicate(),
        }
    }

    /// Create a flight plan for arrivals
    pub fn arrival_plan(dest: String, route: Route, ac_type: String) -> Self {
        Self::new(
            "I".to_string(),
            ac_type,
            250,
            "EDDF".to_string(),
            1130,
            1130,
            36000,
            dest,
            route,
        )
    }

    /// Format as FSD message
    pub fn to_fsd_string(&self) -> String {
        format!(
            ":*A:{}:{}:{}:{}:{}:{}:{}:{}:01:00:0:0::/v/:{}",
            self.flight_rules,
            self.aircraft_type,
            self.enroute_speed,
            self.departure,
            self.off_block_time,
            self.enroute_time,
            self.cruise_altitude,
            self.destination,
            self.route
        )
    }
}

impl std::fmt::Display for FlightPlan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_fsd_string())
    }
}
