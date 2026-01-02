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
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            PlaneMode::FlightPlan => write!(f, "FP"),
            PlaneMode::Heading => write!(f, "HDG"),
            PlaneMode::ILS => write!(f, "ILS"),
            PlaneMode::GroundStationary => write!(f, "GND_STAT"),
            PlaneMode::GroundTaxi => write!(f, "TAXI"),
            PlaneMode::GroundReady => write!(f, "RDY"),
            PlaneMode::None => write!(f, "NONE"),
        }
    }
}
