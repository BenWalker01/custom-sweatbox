use anyhow::Result;
use std::sync::Arc;
use custom_sweatbox_rust::utils::navigation;

#[test]
fn test_load_fixes() -> Result<()> {
    // Load fix database
    let fix_db = navigation::load_navigation_data("data")?;
    
    println!("Loaded {} fixes", fix_db.len());
    
    // Check for known fixes
    assert!(fix_db.get("ABBEW").is_some(), "ABBEW should exist");
    assert!(fix_db.get("EGLL").is_some(), "EGLL airport should exist");
    
    // Verify coordinates
    if let Some((lat, lon)) = fix_db.get("ABBEW") {
        println!("ABBEW at {}, {}", lat, lon);
        assert!((lat - 50.50330).abs() < 0.001);
        assert!((lon - (-3.47601)).abs() < 0.001);
    }
    
    Ok(())
}

#[test]
fn test_plane_with_fixes() -> Result<()> {
    use custom_sweatbox_rust::simulator::{Plane, FlightPlan, Route};
    
    // Load fix database
    let fix_db = Arc::new(navigation::load_navigation_data("data")?);
    
    // Create a simple route
    let route = Route::new(
        "TIMBA LAM BIG".to_string(),
        "EGLL".to_string(),
        Some("EGPF".to_string())
    );
    
    let flight_plan = FlightPlan::new(
        "I".to_string(),
        "B738".to_string(),
        250,
        "EGLL".to_string(),
        1200,
        130,
        36000,
        "EGPF".to_string(),
        route
    );
    
    // Create aircraft at TIMBA
    let plane = Plane::from_fix(
        "TEST123".to_string(),
        "TIMBA".to_string(),
        1234,
        15000.0,
        270.0,
        280.0,
        0.0,
        flight_plan,
        None,
        None,
        Some(fix_db)
    );
    
    // Verify aircraft was created with real coordinates
    println!("Aircraft at {}, {}", plane.lat, plane.lon);
    
    // Should not be at the default position
    assert!(plane.lat != 51.15487 || plane.lon != -0.16454);
    
    Ok(())
}
