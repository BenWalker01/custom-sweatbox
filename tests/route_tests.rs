use custom_sweatbox_rust::simulator::Route;

#[test]
fn test_route_with_sid() {
    // Test route with SID: BPK5K/09L then regular waypoints
    let route = Route::new(
        "BPK5K/09L DVR UL9 KONAN".to_string(),
        "EGLL".to_string(),
        Some("EGPF".to_string()),
    );

    println!("Route string: {}", route.route_string);
    println!("Fixes: {:?}", route.fixes);
    
    // BPK5K/09L should expand to: RW27R D110B D070J D196J D196F BAPAG BPK
    // Then followed by: DVR UL9 KONAN
    assert!(route.fixes.len() > 0);
    assert!(route.fixes.contains(&"BPK".to_string()));
    assert!(route.fixes.contains(&"BAPAG".to_string()));
}

#[test]
fn test_route_with_star() {
    // Test route with STAR at the end
    let route = Route::new(
        "KONAN UL9 DVR ALESO1H/27R".to_string(),
        "EGPF".to_string(),
        Some("EGLL".to_string()),
    );

    println!("Route string: {}", route.route_string);
    println!("Fixes: {:?}", route.fixes);
    
    // ALESO1H/27R should expand to: ALESO ROTNO ETVAX TIGER LLE01 BIG CF27R FI27R RW27R
    assert!(route.fixes.len() > 0);
    assert!(route.fixes.contains(&"ALESO".to_string()));
    assert!(route.fixes.contains(&"TIGER".to_string()));
    assert!(route.fixes.contains(&"RW27R".to_string()));
}

#[test]
fn test_route_with_sid_and_star() {
    // Test route with both SID and STAR
    let route = Route::new(
        "BPK5K/09L DVR UL9 KONAN ALESO1H/27R".to_string(),
        "EGLL".to_string(),
        Some("EGLL".to_string()),
    );

    println!("Route string: {}", route.route_string);
    println!("Fixes: {:?}", route.fixes);
    
    // Should have both SID and STAR expanded
    assert!(route.fixes.len() > 0);
    
    // Check SID fixes
    assert!(route.fixes.contains(&"BPK".to_string()));
    assert!(route.fixes.contains(&"BAPAG".to_string()));
    
    // Check STAR fixes
    assert!(route.fixes.contains(&"ALESO".to_string()));
    assert!(route.fixes.contains(&"TIGER".to_string()));
    
    // Check intermediate waypoints
    assert!(route.fixes.contains(&"DVR".to_string()));
    assert!(route.fixes.contains(&"KONAN".to_string()));
}

#[test]
fn test_route_without_procedures() {
    // Test simple route without SID/STAR
    let route = Route::new(
        "DVR UL9 KONAN UL607 TALLA".to_string(),
        "EGLL".to_string(),
        Some("EGPF".to_string()),
    );

    println!("Route string: {}", route.route_string);
    println!("Fixes: {:?}", route.fixes);
    
    assert!(route.fixes.contains(&"DVR".to_string()));
    assert!(route.fixes.contains(&"KONAN".to_string()));
    assert!(route.fixes.contains(&"TALLA".to_string()));
}
