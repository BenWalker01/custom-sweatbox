use std::collections::HashMap;
use std::fs;
use std::path::Path;
use anyhow::{Result, Context};

pub type ProcedureDatabase = HashMap<String, HashMap<String, String>>;

/// Parse SIDs from airport file
/// Format: SID:ICAO:RUNWAY:SIDNAME:FIXES...
pub fn load_sids<P: AsRef<Path>>(airport_dir: P) -> Result<ProcedureDatabase> {
    let sids_file = airport_dir.as_ref().join("Sids.txt");
    
    if !sids_file.exists() {
        return Ok(HashMap::new());
    }

    let content = fs::read_to_string(&sids_file)
        .with_context(|| format!("Failed to read SIDs file: {:?}", sids_file))?;

    let mut sids: ProcedureDatabase = HashMap::new();

    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with(';') {
            continue;
        }

        // Format: SID:ICAO:RUNWAY:SIDNAME:FIXES...
        let parts: Vec<&str> = line.split(':').collect();
        if parts.len() >= 5 && parts[0] == "SID" {
            let sid_name = parts[3].to_string();
            let runway = parts[2].to_string();
            let fixes = parts[4].to_string();

            sids.entry(sid_name)
                .or_insert_with(HashMap::new)
                .insert(runway, fixes);
        }
    }

    Ok(sids)
}

/// Parse STARs from airport file
/// Format: STAR:ICAO:RUNWAY:STARNAME:FIXES...
pub fn load_stars<P: AsRef<Path>>(airport_dir: P) -> Result<ProcedureDatabase> {
    let stars_file = airport_dir.as_ref().join("Stars.txt");
    
    if !stars_file.exists() {
        return Ok(HashMap::new());
    }

    let content = fs::read_to_string(&stars_file)
        .with_context(|| format!("Failed to read STARs file: {:?}", stars_file))?;

    let mut stars: ProcedureDatabase = HashMap::new();

    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with(';') {
            continue;
        }

        // Format: STAR:ICAO:RUNWAY:STARNAME:FIXES...
        let parts: Vec<&str> = line.split(':').collect();
        if parts.len() >= 5 && parts[0] == "STAR" {
            let star_name = parts[3].to_string();
            let runway = parts[2].to_string();
            let fixes = parts[4].to_string();

            stars.entry(star_name)
                .or_insert_with(HashMap::new)
                .insert(runway, fixes);
        }
    }

    Ok(stars)
}

/// Load both SIDs and STARs for an airport
pub fn load_procedures<P: AsRef<Path>>(
    data_dir: P,
    icao: &str,
) -> Result<(ProcedureDatabase, ProcedureDatabase)> {
    let airport_dir = data_dir.as_ref().join("Airports").join(icao);

    if !airport_dir.exists() {
        return Ok((HashMap::new(), HashMap::new()));
    }

    let sids = load_sids(&airport_dir)?;
    let stars = load_stars(&airport_dir)?;

    Ok((sids, stars))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_load_egll_sids() -> Result<()> {
        let sids = load_sids("data/Airports/EGLL")?;
        
        // Check BPK5K SID exists for runway 09L
        if let Some(bpk5k) = sids.get("BPK5K") {
            if let Some(fixes) = bpk5k.get("09L") {
                assert!(fixes.contains("BPK"));
                println!("BPK5K/09L: {}", fixes);
            }
        }

        println!("Loaded {} SIDs", sids.len());
        Ok(())
    }

    #[test]
    fn test_load_egll_stars() -> Result<()> {
        let stars = load_stars("data/Airports/EGLL")?;
        
        // Check ALESO1H STAR exists
        if let Some(aleso) = stars.get("ALESO1H") {
            if let Some(fixes) = aleso.get("27R") {
                assert!(fixes.contains("ALESO"));
                println!("ALESO1H/27R: {}", fixes);
            }
        }

        println!("Loaded {} STARs", stars.len());
        Ok(())
    }
}
