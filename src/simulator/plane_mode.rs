use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneMode {
    FlightPlan,
    Heading,
    ILS,
    GroundStationary,
    GroundTaxi,
    GroundReady,
    None,
}

impl PlaneMode {
    pub fn is_ground(&self) -> bool {
        matches!(
            self,
            PlaneMode::GroundStationary | PlaneMode::GroundTaxi | PlaneMode::GroundReady
        )
    }

    pub fn is_airborne(&self) -> bool {
        matches!(self, PlaneMode::FlightPlan | PlaneMode::Heading | PlaneMode::ILS)
    }
}

impl fmt::Display for PlaneMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PlaneMode::FlightPlan => write!(f, "FLIGHTPLAN"),
            PlaneMode::Heading => write!(f, "HEADING"),
            PlaneMode::ILS => write!(f, "ILS"),
            PlaneMode::GroundStationary => write!(f, "GROUND_STATIONARY"),
            PlaneMode::GroundTaxi => write!(f, "GROUND_TAXI"),
            PlaneMode::GroundReady => write!(f, "GROUND_READY"),
            PlaneMode::None => write!(f, "NONE"),
        }
    }
}
