use serde::{Deserialize, Serialize};

/// Flight plan information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlightPlan {
    pub aircraft_type: String,
    pub cruise_speed: u32,
    pub departure: String,
    pub arrival: String,
    pub alternate: String,
    pub cruise_altitude: u32,
    pub route: String,
    pub remarks: String,
    pub fuel_hours: u32,
    pub fuel_minutes: u32,
}

impl FlightPlan {
    pub fn new(
        aircraft_type: String,
        departure: String,
        arrival: String,
        cruise_altitude: u32,
        route: String,
    ) -> Self {
        Self {
            aircraft_type: aircraft_type.clone(),
            cruise_speed: 450, // Default, will be updated based on aircraft performance
            departure,
            arrival: arrival.clone(),
            alternate: arrival.clone(), // Use arrival as alternate for now
            cruise_altitude,
            route,
            remarks: "/v/".to_string(),
            fuel_hours: 2,
            fuel_minutes: 30,
        }
    }

    /// Format as FSD flight plan string
    /// Format: *A:RULES:ACFT/EQUIP:TAS:DEP:DEPTIME:ACTUALTIME:ALT:DEST:HRS:MINS:ENDURANCE_HRS:ENDURANCE_MINS:ALT_AIRPORT:REMARKS:ROUTE
    pub fn to_fsd_string(&self) -> String {
        format!(
            "*A:I:{}/H-S/C:{}:{}:0:0:{}:{}:{}:{}:{}:{}:{}:{}:{}",
            self.aircraft_type,
            self.cruise_speed,
            self.departure,
            self.cruise_altitude,
            self.arrival,
            self.fuel_hours,
            self.fuel_minutes,
            self.fuel_hours,
            self.fuel_minutes,
            self.alternate,
            self.remarks,
            self.route
        )
    }
}
