use anyhow::Result;
use rand::Rng;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use tokio::sync::mpsc::{error::TryRecvError, UnboundedReceiver};
use tokio::time::{interval, Duration};
use tracing::{debug, info, warn};

use super::agreement_resolver::AgreementResolver;
use super::ai_controller::AiController;
use super::ai_pilot::AiPilot;
use super::handoff_resolver::{OwnershipDecision, OwnershipResolver};
use crate::aircraft::aircraft::FlightPhase;
use crate::aircraft::Aircraft;
use crate::config::{FleetConfig, SimulationConfig};
use crate::scenario::Scenario;
use crate::utils::navigation::{haversine_nm, heading_from_to, sf_coords_to_decimal, FixDatabase};
use crate::utils::performance::PerformanceDatabase;
use crate::utils::procedures::load_stars;

#[derive(Debug, Clone)]
struct PendingHandoffAcceptance {
    accepting_controller: String,
    from_controller: String,
    due_at: std::time::Instant,
}

#[derive(Debug, Clone)]
struct PendingTransitProfileHandoff {
    preferred_controller: String,
    handoff_fix: String,
    agreed_altitude_ft: i32,
}

/// Main simulation controller
pub struct Simulator {
    scenario: Arc<Scenario>,
    sim_config: Arc<SimulationConfig>,
    fleet_config: Arc<FleetConfig>,
    nav_db: Arc<FixDatabase>,
    perf_db: Arc<PerformanceDatabase>,
    server_addr: String,
    ai_controllers: Vec<AiController>,
    aircraft: Vec<Aircraft>,
    pilot_clients: HashMap<String, AiPilot>,
    controller_message_rxs: Vec<UnboundedReceiver<String>>,
    agreement_resolver: Option<AgreementResolver>,
    ownership_resolver: Option<OwnershipResolver>,
    pending_handoffs: HashMap<String, String>,
    pending_handoff_accepts: HashMap<String, PendingHandoffAcceptance>,
    pending_transit_handoffs: HashMap<String, PendingTransitProfileHandoff>,
    running: bool,
    squawk_pool: Vec<u16>,
    used_callsigns: HashSet<String>,
}

impl Simulator {
    /// Create a new simulator
    pub fn new(
        scenario: Scenario,
        sim_config: SimulationConfig,
        fleet_config: FleetConfig,
        nav_db: Arc<FixDatabase>,
        perf_db: Arc<PerformanceDatabase>,
        server_addr: String,
    ) -> Self {
        Self {
            scenario: Arc::new(scenario),
            sim_config: Arc::new(sim_config),
            fleet_config: Arc::new(fleet_config),
            nav_db,
            perf_db,
            server_addr,
            ai_controllers: Vec::new(),
            aircraft: Vec::new(),
            pilot_clients: HashMap::new(),
            controller_message_rxs: Vec::new(),
            agreement_resolver: None,
            ownership_resolver: None,
            pending_handoffs: HashMap::new(),
            pending_handoff_accepts: HashMap::new(),
            pending_transit_handoffs: HashMap::new(),
            running: false,
            squawk_pool: crate::config::get_ccams_squawks(),
            used_callsigns: HashSet::new(),
        }
    }

    /// Initialize the simulation
    pub async fn initialize(&mut self) -> Result<()> {
        info!("[SIMULATOR] Initializing simulation...");

        // Display scenario information
        let stats = self.scenario.statistics();
        info!("{}", stats);

        // Build sector ownership resolver used for controller ownership and handoffs.
        self.initialize_ownership_resolver()?;
        self.initialize_agreement_resolver()?;

        // Login AI controllers
        self.login_ai_controllers().await?;

        info!("[SIMULATOR] Initialization complete");
        Ok(())
    }

    fn initialize_ownership_resolver(&mut self) -> Result<()> {
        let profile = &self.scenario.config;
        let resolver = OwnershipResolver::from_scenario_data(
            &profile.active_aerodromes,
            &profile.active_runways,
            &profile.active_controllers,
            &profile.master_controller,
            &profile.other_controllers,
            &profile.inactive_sectors,
            &self.nav_db,
        )?;
        self.ownership_resolver = Some(resolver);
        Ok(())
    }

    fn initialize_agreement_resolver(&mut self) -> Result<()> {
        let resolver = AgreementResolver::load_from_dir("data/Agreements/Internal")?;
        self.agreement_resolver = Some(resolver);
        Ok(())
    }

    /// Login AI controllers to the FSD server
    async fn login_ai_controllers(&mut self) -> Result<()> {
        info!("[SIMULATOR] Logging in AI controllers...");

        self.ai_controllers.clear();
        self.controller_message_rxs.clear();

        let mut controller_positions = Vec::new();
        let mut seen = HashSet::new();

        let (primary_callsign, primary_freq) = self.scenario.master_controller();
        let primary_key = primary_callsign.trim().to_uppercase();
        if !primary_key.is_empty() && seen.insert(primary_key) {
            controller_positions.push((primary_callsign.to_string(), primary_freq.to_string()));
        }

        for (callsign, freq) in self.scenario.other_controllers() {
            let key = callsign.trim().to_uppercase();
            if key.is_empty() || !seen.insert(key) {
                continue;
            }
            controller_positions.push((callsign.clone(), freq.clone()));
        }

        for (callsign, freq) in controller_positions {
            info!("[SIMULATOR] Creating controller: {} on {}", callsign, freq);

            let mut controller = AiController::new(callsign.clone(), freq, 51.5, -0.5, 300);

            controller.connect(&self.server_addr).await?;
            tokio::time::sleep(Duration::from_millis(200)).await;
            controller.login().await?;
            tokio::time::sleep(Duration::from_millis(300)).await;
            controller.send_ip_query().await?;

            if let Some(rx) = controller.start_message_loop(true).await? {
                self.controller_message_rxs.push(rx);
            }

            self.ai_controllers.push(controller);
            info!("[SIMULATOR] Controller {} logged in", callsign);
        }

        info!(
            "[SIMULATOR] {} AI controllers logged in",
            self.ai_controllers.len()
        );

        Ok(())
    }

    /// Start the main simulation loop
    pub async fn run(&mut self, shutdown: tokio::sync::broadcast::Receiver<()>) -> Result<()> {
        info!("[SIMULATOR] Starting main simulation loop...");
        self.running = true;

        // Create timers for different spawn intervals
        let mut departure_timers = self.create_departure_timers();
        let mut transit_timers = self.create_transit_timers();

        // Main update loop (runs at radar update rate)
        let radar_update_ms = (1000.0 / self.sim_config.radar_update_rate) as u64;
        let mut update_interval = interval(Duration::from_millis(radar_update_ms));

        let mut loop_count = 0u64;
        let mut shutdown_rx = shutdown;

        loop {
            tokio::select! {
                _ = shutdown_rx.recv() => {
                    info!("[SIMULATOR] Shutdown signal received");
                    break;
                }
                _ = update_interval.tick() => {
                    loop_count += 1;

                    let delta_time = (radar_update_ms as f64) / 1000.0;

                    // Check departure timers
                    self.check_departure_spawns(&mut departure_timers, loop_count).await?;

                    // Check transit timers
                    self.check_transit_spawns(&mut transit_timers, loop_count).await?;

                    // Apply controller-issued commands to simulated aircraft
                    self.process_controller_messages().await?;

                    // Update all aircraft
                    self.update_aircraft(delta_time);

                    // Apply profile-defined pre-handoff behavior for transits.
                    self.process_profile_transit_handoffs();

                    // Apply automatic sector-based handoff logic for AI-owned aircraft.
                    self.process_automatic_handoffs().await?;

                    // Apply delayed AI handoff accepts (10-30s delay).
                    self.process_pending_handoff_accepts();

                    // Send pilot position updates every 5 seconds (25 ticks at 5 Hz)
                    if loop_count % 25 == 0 {
                        self.broadcast_pilot_positions().await?;
                    }

                    // Log status periodically
                    if loop_count % 50 == 0 {
                        debug!("[SIMULATOR] Loop {}: {} controllers, {} aircraft",
                               loop_count, self.ai_controllers.len(), self.aircraft.len());
                    }
                }
            }
        }

        self.running = false;
        info!("[SIMULATOR] Simulation loop stopped");
        Ok(())
    }

    async fn process_controller_messages(&mut self) -> Result<()> {
        let mut buffered_messages = Vec::new();
        let mut disconnected_indexes = Vec::new();

        for (index, rx) in self.controller_message_rxs.iter_mut().enumerate() {
            loop {
                match rx.try_recv() {
                    Ok(message) => buffered_messages.push(message),
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        disconnected_indexes.push(index);
                        break;
                    }
                }
            }
        }

        disconnected_indexes.sort_unstable();
        disconnected_indexes.dedup();
        for index in disconnected_indexes.into_iter().rev() {
            warn!("[SIMULATOR] Controller message channel disconnected");
            self.controller_message_rxs.swap_remove(index);
        }

        for message in buffered_messages {
            if message.starts_with("$CQ") && message.contains("@94835") {
                info!("[SIMULATOR CTRL RX] {}", message);
            } else if message.starts_with("$HA") || message.starts_with("$HO") {
                info!("[SIMULATOR HANDOFF RX] {}", message);
            }
            self.handle_controller_message(&message).await?;
        }

        Ok(())
    }

    async fn handle_controller_message(&mut self, message: &str) -> Result<()> {
        if message.starts_with("$HO") {
            self.handle_handoff_offer(message);
            return Ok(());
        }

        if message.starts_with("$HA") {
            self.handle_handoff_accept(message);
            return Ok(());
        }

        if !message.starts_with("$CQ") {
            return Ok(());
        }

        let parts: Vec<&str> = message.split(':').collect();
        if parts.len() < 4 {
            return Ok(());
        }

        let from_controller = parts[0].trim_start_matches("$CQ");
        let target = parts[1];
        let command = parts[2];

        // Sim command channel used by legacy sweatbox control messages.
        if target != "@94835" {
            if command == "PD"
                || command == "ROND"
                || command == "SC"
                || command == "TA"
                || command == "IT"
                || command == "BC"
                || command == "DR"
            {
                info!("[SIMULATOR CTRL DROP] target {} for {}", target, message);
            }
            return Ok(());
        }

        let mut remove_callsign: Option<String> = None;

        match command {
            "IT" => {
                let callsign = parts[3];
                if let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) {
                    if let Some(owner) = self.aircraft[index].assumed_by.as_deref() {
                        if !owner.eq_ignore_ascii_case(from_controller.trim()) {
                            warn!(
                                "[SIMULATOR] {} cannot assume {}; currently assumed by {}",
                                from_controller, callsign, owner
                            );
                            return Ok(());
                        }
                    }

                    self.aircraft[index].set_assumed_by(Some(from_controller.trim().to_string()));
                    info!("[SIMULATOR] {} assumed by {}", callsign, from_controller);
                }
            }
            "HT" => {
                if parts.len() < 5 {
                    return Ok(());
                }

                let callsign = parts[3].trim();
                let owner = parts[4].trim();

                if let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) {
                    if owner.is_empty() {
                        self.aircraft[index].set_assumed_by(None);
                        info!("[SIMULATOR] Cleared ownership marker for {}", callsign);
                    } else {
                        self.aircraft[index].set_assumed_by(Some(owner.to_string()));
                        info!("[SIMULATOR] Ownership marker {} -> {}", callsign, owner);
                    }
                }
            }
            "WH" => {
                if parts.len() < 4 {
                    return Ok(());
                }

                let callsign = parts[3].trim();
                if let Some(owner) = self
                    .aircraft
                    .iter()
                    .find(|a| a.callsign == callsign)
                    .and_then(|a| a.assumed_by.clone())
                    .filter(|owner| self.is_ai_controller(owner))
                {
                    if self.send_assumed_tag_marker(&owner, callsign) {
                        info!(
                            "[SIMULATOR] Re-marked assumed tag {} for {}",
                            owner, callsign
                        );
                    }
                }
            }
            "SC" => {
                if parts.len() < 5 {
                    return Ok(());
                }

                let callsign = parts[3];
                let instruction = parts[4];

                if let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) {
                    if !Self::controller_has_assumed_tag(&self.aircraft[index], from_controller) {
                        warn!(
                            "[SIMULATOR] {} ignored SC for {} (tag not assumed)",
                            from_controller, callsign
                        );
                        return Ok(());
                    }

                    if let Some(value) = instruction.strip_prefix('H') {
                        if let Ok(target_heading) = value.parse::<i32>() {
                            self.aircraft[index].assign_heading(target_heading);
                            info!("[SIMULATOR] {} heading {}", callsign, target_heading);
                        }
                    } else if let Some(value) = instruction.strip_prefix('S') {
                        if let Ok(target_speed) = value.parse::<u32>() {
                            self.aircraft[index].assign_speed(target_speed);
                            info!("[SIMULATOR] {} speed {}", callsign, target_speed);
                        }
                    } else if let Some(value) = instruction.strip_prefix('M') {
                        if let Ok(mach_times_100) = value.parse::<u32>() {
                            let ias = (((mach_times_100 as f64) / 100.0) * (450.0 / 0.7842)).round()
                                as u32;
                            self.aircraft[index].assign_speed(ias);
                            info!(
                                "[SIMULATOR] {} speed M{} (~{}kt)",
                                callsign, mach_times_100, ias
                            );
                        }
                    } else {
                        // Some clients encode direct/route payloads via SC:<callsign>:<FIX>
                        self.apply_route_payload(callsign, instruction, from_controller)?;
                    }
                }
            }
            "TA" => {
                if parts.len() < 5 {
                    return Ok(());
                }

                let callsign = parts[3];
                if let Ok(mut target_altitude) = parts[4].parse::<i32>() {
                    if target_altitude == 1 {
                        self.apply_route_payload(callsign, "ILS", from_controller)?;
                        return Ok(());
                    }

                    if let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) {
                        if !Self::controller_has_assumed_tag(&self.aircraft[index], from_controller)
                        {
                            warn!(
                                "[SIMULATOR] {} ignored TA for {} (tag not assumed)",
                                from_controller, callsign
                            );
                            return Ok(());
                        }

                        if target_altitude == 0 {
                            target_altitude =
                                self.aircraft[index].flight_plan.cruise_altitude as i32 * 100;
                        }

                        let aircraft = &mut self.aircraft[index];
                        aircraft.assign_altitude(target_altitude);
                        info!("[SIMULATOR] {} altitude {}", callsign, target_altitude);
                    }
                }
            }
            "BC" => {
                if parts.len() < 5 {
                    return Ok(());
                }

                let callsign = parts[3];
                let squawk = parts[4];

                if squawk == "7000" {
                    return Ok(());
                }

                if let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) {
                    if !Self::controller_has_assumed_tag(&self.aircraft[index], from_controller) {
                        warn!(
                            "[SIMULATOR] {} ignored BC for {} (tag not assumed)",
                            from_controller, callsign
                        );
                        return Ok(());
                    }

                    let aircraft = &mut self.aircraft[index];
                    aircraft.squawk = squawk.to_string();
                    info!("[SIMULATOR] {} squawk {}", callsign, squawk);
                }
            }
            "DR" => {
                let callsign = parts[3];
                if let Some(aircraft) = self.aircraft.iter().find(|a| a.callsign == callsign) {
                    if !Self::controller_has_assumed_tag(aircraft, from_controller) {
                        warn!(
                            "[SIMULATOR] {} ignored DR for {} (tag not assumed)",
                            from_controller, callsign
                        );
                        return Ok(());
                    }
                    remove_callsign = Some(callsign.to_string());
                }
            }
            "PD" | "ROND" => {
                if parts.len() >= 5 {
                    let callsign = parts[3];
                    let payload = Self::payload_from_parts(&parts, 4);
                    self.apply_route_payload(callsign, &payload, from_controller)?;
                }
            }
            _ => {
                // Handle direct-to style packets, e.g. ...:<callsign>:DVR
                if parts.len() >= 5 {
                    let callsign = parts[3];
                    let payload = Self::payload_from_parts(&parts, 4);
                    self.apply_route_payload(callsign, &payload, from_controller)?;
                }
            }
        }

        if let Some(callsign) = remove_callsign {
            self.remove_aircraft_by_callsign(&callsign).await?;
        }

        Ok(())
    }

    async fn remove_aircraft_by_callsign(&mut self, callsign: &str) -> Result<()> {
        let removed_squawk = self
            .aircraft
            .iter()
            .find(|a| a.callsign == callsign)
            .and_then(|a| a.squawk.parse::<u16>().ok());

        let before = self.aircraft.len();
        self.aircraft.retain(|a| a.callsign != callsign);

        if before == self.aircraft.len() {
            return Ok(());
        }

        self.used_callsigns.remove(callsign);
        if let Some(squawk) = removed_squawk {
            self.squawk_pool.push(squawk);
        }

        if let Some(mut pilot) = self.pilot_clients.remove(callsign) {
            pilot.disconnect().await?;
        }

        self.pending_handoffs.remove(callsign);
        self.pending_handoff_accepts.remove(callsign);
        self.pending_transit_handoffs.remove(callsign);

        info!("[SIMULATOR] Removed aircraft {}", callsign);
        Ok(())
    }

    fn resolve_ownership(&self, aircraft: &Aircraft) -> Option<OwnershipDecision> {
        self.ownership_resolver
            .as_ref()
            .and_then(|resolver| resolver.resolve_owner_for_aircraft(aircraft))
    }

    fn is_ai_controller(&self, callsign: &str) -> bool {
        self.ai_controllers
            .iter()
            .any(|controller| controller.callsign().eq_ignore_ascii_case(callsign.trim()))
    }

    fn is_controller_online(&self, callsign: &str) -> bool {
        self.is_ai_controller(callsign)
    }

    fn select_transit_fallback_owner(&self, resolved_owner: Option<&String>) -> Option<String> {
        if self.is_controller_online("LON_E_CTR") {
            return Some("LON_E_CTR".to_string());
        }

        if let Some(owner) = resolved_owner.filter(|owner| self.is_ai_controller(owner)) {
            return Some(owner.clone());
        }

        self.ai_controllers
            .first()
            .map(|controller| controller.callsign().to_string())
    }

    fn send_from_ai_controller(&self, callsign: &str, message: &str) -> bool {
        let Some(controller) = self
            .ai_controllers
            .iter()
            .find(|controller| controller.callsign().eq_ignore_ascii_case(callsign.trim()))
        else {
            return false;
        };

        if let Err(error) = controller.send_message(message) {
            warn!(
                "[SIMULATOR] Failed to queue message from {} ({}): {}",
                callsign, message, error
            );
            return false;
        }

        true
    }

    fn send_handoff_offer(
        &self,
        from_controller: &str,
        to_controller: &str,
        callsign: &str,
    ) -> bool {
        let message = format!(
            "$HO{}:{}:{}",
            from_controller.trim(),
            to_controller.trim(),
            callsign.trim()
        );
        self.send_from_ai_controller(from_controller, &message)
    }

    fn send_handoff_accept(
        &self,
        accepting_controller: &str,
        from_controller: &str,
        callsign: &str,
    ) -> bool {
        let message = format!(
            "$HA{}:{}:{}",
            accepting_controller.trim(),
            from_controller.trim(),
            callsign.trim()
        );
        self.send_from_ai_controller(accepting_controller, &message)
    }

    fn send_assumed_tag_marker(&self, owner_controller: &str, callsign: &str) -> bool {
        let owner_controller = owner_controller.trim();
        let callsign = callsign.trim();
        if owner_controller.is_empty() || callsign.is_empty() {
            return false;
        }
        if !self.is_ai_controller(owner_controller) {
            return false;
        }

        // Legacy EuroScope ownership marker payload.
        let message = format!(
            "$CQ{}:@94835:HT:{}:{}",
            owner_controller, callsign, owner_controller
        );
        self.send_from_ai_controller(owner_controller, &message)
    }

    fn schedule_handoff_accept(
        &mut self,
        accepting_controller: &str,
        from_controller: &str,
        callsign: &str,
    ) {
        let callsign = callsign.trim();
        let accepting_controller = accepting_controller.trim();
        let from_controller = from_controller.trim();
        if callsign.is_empty() || accepting_controller.is_empty() {
            return;
        }

        if self
            .pending_handoff_accepts
            .get(callsign)
            .is_some_and(|pending| {
                pending
                    .accepting_controller
                    .eq_ignore_ascii_case(accepting_controller)
                    && pending
                        .from_controller
                        .eq_ignore_ascii_case(from_controller)
            })
        {
            return;
        }

        let delay_secs = rand::thread_rng().gen_range(10..=30);
        let due_at = std::time::Instant::now() + std::time::Duration::from_secs(delay_secs);

        self.pending_handoff_accepts.insert(
            callsign.to_string(),
            PendingHandoffAcceptance {
                accepting_controller: accepting_controller.to_string(),
                from_controller: from_controller.to_string(),
                due_at,
            },
        );

        info!(
            "[SIMULATOR] Scheduled delayed HA {} -> {} for {} in {}s",
            from_controller, accepting_controller, callsign, delay_secs
        );
    }

    fn process_pending_handoff_accepts(&mut self) {
        let now = std::time::Instant::now();
        let ready: Vec<String> = self
            .pending_handoff_accepts
            .iter()
            .filter_map(|(callsign, pending)| {
                if pending.due_at <= now {
                    Some(callsign.clone())
                } else {
                    None
                }
            })
            .collect();

        for callsign in ready {
            let Some(mut pending) = self.pending_handoff_accepts.remove(&callsign) else {
                continue;
            };

            if self.send_handoff_accept(
                &pending.accepting_controller,
                &pending.from_controller,
                &callsign,
            ) {
                self.pending_handoffs.remove(&callsign);
                info!(
                    "[SIMULATOR] Sent delayed HA {} -> {} for {}",
                    pending.from_controller, pending.accepting_controller, callsign
                );
            } else {
                pending.due_at = std::time::Instant::now() + std::time::Duration::from_secs(5);
                self.pending_handoff_accepts.insert(callsign, pending);
            }
        }
    }

    fn process_profile_transit_handoffs(&mut self) {
        let pending_callsigns: Vec<String> =
            self.pending_transit_handoffs.keys().cloned().collect();

        for callsign in pending_callsigns {
            let Some(plan) = self.pending_transit_handoffs.get(&callsign).cloned() else {
                continue;
            };

            let Some(index) = self
                .aircraft
                .iter()
                .position(|aircraft| aircraft.callsign == callsign)
            else {
                self.pending_transit_handoffs.remove(&callsign);
                continue;
            };

            let Some(current_owner) = self.aircraft[index].assumed_by.clone() else {
                continue;
            };
            if !self.is_ai_controller(&current_owner) {
                continue;
            }

            if (self.aircraft[index].altitude - plan.agreed_altitude_ft).abs() > 300 {
                self.aircraft[index].assign_altitude(plan.agreed_altitude_ft);
                continue;
            }

            let Some((fix_lat, fix_lon)) = self.nav_db.get(&plan.handoff_fix) else {
                warn!(
                    "[SIMULATOR] Transit {} missing handoff fix {}, dropping fallback handoff plan",
                    callsign, plan.handoff_fix
                );
                self.pending_transit_handoffs.remove(&callsign);
                continue;
            };

            let distance_nm = haversine_nm(
                self.aircraft[index].latitude,
                self.aircraft[index].longitude,
                *fix_lat,
                *fix_lon,
            );
            if distance_nm > 10.0 {
                continue;
            }

            let target_controller = if !plan.preferred_controller.is_empty()
                && self.is_controller_online(&plan.preferred_controller)
            {
                plan.preferred_controller.clone()
            } else {
                self.resolve_ownership(&self.aircraft[index])
                    .map(|decision| decision.owner_callsign)
                    .or_else(|| {
                        if plan.preferred_controller.is_empty() {
                            None
                        } else {
                            Some(plan.preferred_controller.clone())
                        }
                    })
                    .unwrap_or_default()
            };

            if target_controller.is_empty()
                || current_owner.eq_ignore_ascii_case(&target_controller)
            {
                self.pending_transit_handoffs.remove(&callsign);
                continue;
            }

            if self.send_handoff_offer(&current_owner, &target_controller, &callsign) {
                self.aircraft[index].set_assumed_by(Some(target_controller.clone()));
                self.send_assumed_tag_marker(&target_controller, &callsign);
                self.pending_handoffs
                    .insert(callsign.clone(), target_controller.clone());
                self.pending_transit_handoffs.remove(&callsign);
                info!(
                    "[SIMULATOR] Transit pre-handoff {} -> {} for {} near {}",
                    current_owner, target_controller, callsign, plan.handoff_fix
                );

                if self.is_ai_controller(&target_controller) {
                    self.schedule_handoff_accept(&target_controller, &current_owner, &callsign);
                }
            }
        }
    }

    fn handle_handoff_offer(&mut self, message: &str) {
        let parts: Vec<&str> = message.split(':').collect();
        if parts.len() < 3 {
            return;
        }

        let from_controller = parts[0].trim_start_matches("$HO").trim();
        let to_controller = parts[1].trim();
        let callsign = parts[2].trim();
        if callsign.is_empty() || to_controller.is_empty() {
            return;
        }

        // AI controllers only auto-accept when the target controller actually owns
        // the aircraft's current sector.
        if !self.is_ai_controller(to_controller) {
            return;
        }

        let Some(index) = self
            .aircraft
            .iter()
            .position(|aircraft| aircraft.callsign == callsign)
        else {
            return;
        };

        if self.aircraft[index]
            .assumed_by
            .as_deref()
            .is_some_and(|owner| owner.eq_ignore_ascii_case(to_controller))
        {
            return;
        }

        if let Some(decision) = self.resolve_ownership(&self.aircraft[index]) {
            if !decision.owner_callsign.eq_ignore_ascii_case(to_controller) {
                debug!(
                    "[SIMULATOR] Accepting HO {} -> {} for {} despite resolver owner {}",
                    from_controller, to_controller, callsign, decision.owner_callsign
                );
            }
        }

        self.schedule_handoff_accept(to_controller, from_controller, callsign);
        self.aircraft[index].set_assumed_by(Some(to_controller.to_string()));
        self.send_assumed_tag_marker(to_controller, callsign);
        self.pending_handoffs.remove(callsign);
        info!(
            "[SIMULATOR] Accepted tag on HO {} -> {} for {} (HA delayed)",
            from_controller, to_controller, callsign
        );
    }

    fn handle_handoff_accept(&mut self, message: &str) {
        let parts: Vec<&str> = message.split(':').collect();
        if parts.len() < 3 {
            return;
        }

        let accepting_controller = parts[0].trim_start_matches("$HA").trim();
        let callsign = parts[2].trim();
        if callsign.is_empty() {
            return;
        }

        let mut assumed_owner: Option<String> = None;
        if let Some(aircraft) = self
            .aircraft
            .iter_mut()
            .find(|aircraft| aircraft.callsign == callsign)
        {
            if accepting_controller.is_empty() {
                aircraft.set_assumed_by(None);
            } else {
                aircraft.set_assumed_by(Some(accepting_controller.to_string()));
                assumed_owner = Some(accepting_controller.to_string());
            }
        }
        if let Some(owner) = assumed_owner {
            self.send_assumed_tag_marker(&owner, callsign);
        }

        self.pending_handoffs.remove(callsign);
        self.pending_handoff_accepts.remove(callsign);
        self.pending_transit_handoffs.remove(callsign);
    }

    async fn process_automatic_handoffs(&mut self) -> Result<()> {
        for index in 0..self.aircraft.len() {
            let callsign = self.aircraft[index].callsign.clone();
            if self.pending_transit_handoffs.contains_key(&callsign) {
                // Keep ownership stable while profile transit fallback manages descent
                // and pre-handoff near the agreed fix.
                continue;
            }

            let expected_owner = self
                .resolve_ownership(&self.aircraft[index])
                .map(|decision| decision.owner_callsign);

            let Some(target_owner) = expected_owner else {
                self.pending_handoffs.remove(&callsign);
                continue;
            };

            let current_owner = self.aircraft[index].assumed_by.clone();

            if current_owner
                .as_deref()
                .is_some_and(|owner| owner.eq_ignore_ascii_case(&target_owner))
            {
                self.pending_handoffs.remove(&callsign);
                continue;
            }

            if current_owner.is_none() {
                // Departures must be explicitly assumed (IT/HA), not auto-assumed.
                self.pending_handoffs.remove(&callsign);
                continue;
            }

            let from_owner = current_owner.unwrap_or_default();
            if !self.is_ai_controller(&from_owner) {
                continue;
            }

            let already_pending = self
                .pending_handoffs
                .get(&callsign)
                .is_some_and(|pending| pending.eq_ignore_ascii_case(&target_owner));
            if already_pending {
                continue;
            }

            let handoff_started = self.send_handoff_offer(&from_owner, &target_owner, &callsign);
            if handoff_started {
                self.aircraft[index].set_assumed_by(Some(target_owner.clone()));
                self.send_assumed_tag_marker(&target_owner, &callsign);
                self.pending_handoffs
                    .insert(callsign.clone(), target_owner.clone());
                info!(
                    "[SIMULATOR] Offered HO {} -> {} for {}",
                    from_owner, target_owner, callsign
                );
            }

            if handoff_started && self.is_ai_controller(&target_owner) {
                self.schedule_handoff_accept(&target_owner, &from_owner, &callsign);
                info!(
                    "[SIMULATOR] Scheduled AI HA for HO {} -> {} ({})",
                    from_owner, target_owner, callsign
                );
            }
        }

        Ok(())
    }

    fn controller_has_assumed_tag(aircraft: &Aircraft, controller: &str) -> bool {
        aircraft
            .assumed_by
            .as_deref()
            .map(str::trim)
            .is_some_and(|owner| owner.eq_ignore_ascii_case(controller.trim()))
    }

    fn payload_from_parts(parts: &[&str], start_index: usize) -> String {
        if start_index >= parts.len() {
            String::new()
        } else {
            parts[start_index..]
                .iter()
                .map(|segment| segment.trim())
                .filter(|segment| !segment.is_empty())
                .collect::<Vec<_>>()
                .join(" ")
        }
    }

    fn apply_route_payload(
        &mut self,
        callsign: &str,
        payload: &str,
        from_controller: &str,
    ) -> Result<()> {
        let payload = payload.trim();
        if payload.is_empty() {
            return Ok(());
        }

        if payload.to_ascii_uppercase().starts_with("HOLD") {
            let hold_fix = payload
                .split(|c: char| !c.is_ascii_alphabetic())
                .filter(|token| !token.is_empty())
                .map(|token| token.to_uppercase())
                .find(|token| token != "HOLD" && token != "AT");

            return self.assign_hold(callsign, hold_fix.as_deref(), from_controller);
        }

        if payload.to_ascii_uppercase().starts_with("STAR") {
            let star_name = payload.get(4..).unwrap_or("").trim();

            if star_name.is_empty() {
                warn!(
                    "[SIMULATOR] {} ignored STAR for {} (missing STAR name)",
                    from_controller, callsign
                );
                return Ok(());
            }

            return self.assign_star(callsign, star_name, from_controller);
        }

        if payload.eq_ignore_ascii_case("ILS") {
            return self.assign_ils(callsign, from_controller);
        }

        let normalized_payload = payload.strip_prefix("LVL").unwrap_or(payload).trim();

        let direct_fix = normalized_payload
            .split(|c: char| !c.is_ascii_alphabetic())
            .filter(|token| token.len() >= 2)
            .map(|token| token.to_uppercase())
            .find(|token| {
                token != "DCT"
                    && token != "DIRECT"
                    && token != "NAV"
                    && token != "OWN"
                    && self.nav_db.contains_key(token)
            })
            .unwrap_or_default();

        if direct_fix.is_empty() {
            warn!(
                "[SIMULATOR] {} ignored direct payload '{}' for {} (no usable fix)",
                from_controller, payload, callsign
            );
            return Ok(());
        }

        if let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) {
            if !Self::controller_has_assumed_tag(&self.aircraft[index], from_controller) {
                warn!(
                    "[SIMULATOR] {} ignored direct for {} (tag not assumed)",
                    from_controller, callsign
                );
                return Ok(());
            }

            let aircraft = &mut self.aircraft[index];
            if aircraft.direct_to_fix(&direct_fix, &self.nav_db) {
                info!("[SIMULATOR] {} direct {}", callsign, direct_fix);
            } else {
                warn!(
                    "[SIMULATOR] {} ignored direct {} for {} (fix unavailable)",
                    from_controller, direct_fix, callsign
                );
            }
        }

        Ok(())
    }

    fn assign_hold(
        &mut self,
        callsign: &str,
        requested_fix: Option<&str>,
        from_controller: &str,
    ) -> Result<()> {
        let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) else {
            return Ok(());
        };

        if !Self::controller_has_assumed_tag(&self.aircraft[index], from_controller) {
            warn!(
                "[SIMULATOR] {} ignored HOLD for {} (tag not assumed)",
                from_controller, callsign
            );
            return Ok(());
        }

        let selected_fix = if let Some(fix) = requested_fix {
            Some(fix.to_uppercase())
        } else {
            let aircraft = &self.aircraft[index];
            aircraft
                .route_fixes
                .get(aircraft.current_fix_index..)
                .and_then(|remaining| remaining.last().cloned())
                .or_else(|| aircraft.route_fixes.last().cloned())
        };

        let Some(hold_fix) = selected_fix else {
            warn!(
                "[SIMULATOR] {} ignored HOLD for {} (no hold fix available)",
                from_controller, callsign
            );
            return Ok(());
        };

        let aircraft = &mut self.aircraft[index];
        if aircraft.assign_hold(&hold_fix, &self.nav_db) {
            info!("[SIMULATOR] {} hold armed at {}", callsign, hold_fix);
        } else {
            warn!(
                "[SIMULATOR] {} ignored HOLD {} for {} (fix unavailable)",
                from_controller, hold_fix, callsign
            );
        }

        Ok(())
    }

    fn resolve_star_fixes(
        &self,
        destination: &str,
        star_name: &str,
    ) -> Result<Option<(Vec<String>, String)>> {
        let Some(active_runway) = self
            .scenario
            .active_runway(destination)
            .map(|r| r.to_string())
        else {
            return Ok(None);
        };

        let stars = load_stars(format!("data/Airports/{}", destination))?;
        if stars.is_empty() {
            return Ok(None);
        }

        let requested_star = star_name.trim().to_uppercase();
        let Some((_, star_runways)) = stars.iter().find(|(name, _)| {
            let normalized = name.trim_start_matches('#');
            normalized.eq_ignore_ascii_case(&requested_star)
        }) else {
            return Ok(None);
        };

        let star_fixes_raw = if let Some(fixes) = star_runways
            .iter()
            .find(|(runway, _)| runway.eq_ignore_ascii_case(&active_runway))
            .map(|(_, fixes)| fixes.clone())
        {
            fixes
        } else if let Some((_, fixes)) = star_runways.iter().next() {
            fixes.clone()
        } else {
            return Ok(None);
        };

        let star_fixes: Vec<String> = star_fixes_raw
            .split_whitespace()
            .filter(|token| !token.starts_with('#'))
            .map(|token| token.to_uppercase())
            .collect();

        if star_fixes.is_empty() {
            return Ok(None);
        }

        Ok(Some((star_fixes, active_runway)))
    }

    fn assign_star(
        &mut self,
        callsign: &str,
        star_name: &str,
        from_controller: &str,
    ) -> Result<()> {
        let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) else {
            return Ok(());
        };

        if !Self::controller_has_assumed_tag(&self.aircraft[index], from_controller) {
            warn!(
                "[SIMULATOR] {} ignored STAR {} for {} (tag not assumed)",
                from_controller, star_name, callsign
            );
            return Ok(());
        }

        let destination = self.aircraft[index].flight_plan.arrival.clone();
        let requested_star = star_name.trim().to_uppercase();
        let Some((star_fixes, active_runway)) =
            self.resolve_star_fixes(&destination, &requested_star)?
        else {
            warn!(
                "[SIMULATOR] STAR {} unavailable for {} ({} ignored)",
                requested_star, destination, callsign
            );
            return Ok(());
        };

        let aircraft = &mut self.aircraft[index];
        let added = aircraft.append_route_fixes(star_fixes);
        info!(
            "[SIMULATOR] {} STAR {} appended {} fixes for runway {}",
            callsign, requested_star, added, active_runway
        );

        Ok(())
    }

    fn assign_ils(&mut self, callsign: &str, from_controller: &str) -> Result<()> {
        let Some(index) = self.aircraft.iter().position(|a| a.callsign == callsign) else {
            return Ok(());
        };

        if !Self::controller_has_assumed_tag(&self.aircraft[index], from_controller) {
            warn!(
                "[SIMULATOR] {} ignored ILS for {} (tag not assumed)",
                from_controller, callsign
            );
            return Ok(());
        }

        let destination = self.aircraft[index].flight_plan.arrival.clone();
        let Some(active_runway) = self
            .scenario
            .active_runway(&destination)
            .map(|r| r.to_string())
        else {
            warn!(
                "[SIMULATOR] No active runway for destination {} ({} ILS ignored)",
                destination, callsign
            );
            return Ok(());
        };

        let Some((runway_heading, threshold_lat, threshold_lon)) =
            self.lookup_runway_endpoint(&destination, &active_runway)?
        else {
            warn!(
                "[SIMULATOR] Could not resolve runway {} for {} ({} ILS ignored)",
                active_runway, destination, callsign
            );
            return Ok(());
        };

        let runway_elevation_ft = self
            .sim_config
            .airport_elevations
            .get(&destination)
            .copied()
            .unwrap_or(0) as i32;

        let aircraft = &mut self.aircraft[index];
        aircraft.assign_ils(
            active_runway.clone(),
            (threshold_lat, threshold_lon),
            runway_heading,
            runway_elevation_ft,
        );

        info!(
            "[SIMULATOR] {} ILS {} {}",
            callsign, destination, active_runway
        );
        Ok(())
    }

    fn lookup_runway_endpoint(
        &self,
        airport: &str,
        runway: &str,
    ) -> Result<Option<(i32, f64, f64)>> {
        let runway_path = format!("data/Airports/{}/Runway.txt", airport);
        let Ok(content) = std::fs::read_to_string(&runway_path) else {
            return Ok(None);
        };

        for line in content.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with(';') {
                continue;
            }

            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 8 {
                continue;
            }

            let rwy_a = parts[0];
            let rwy_b = parts[1];

            let heading_a = match parts[2].parse::<i32>() {
                Ok(v) => v,
                Err(_) => continue,
            };
            let heading_b = match parts[3].parse::<i32>() {
                Ok(v) => v,
                Err(_) => continue,
            };

            if runway.eq_ignore_ascii_case(rwy_a) {
                if let Ok((lat, lon)) = sf_coords_to_decimal(parts[4], parts[5]) {
                    return Ok(Some((heading_a, lat, lon)));
                }
            }

            if runway.eq_ignore_ascii_case(rwy_b) {
                if let Ok((lat, lon)) = sf_coords_to_decimal(parts[6], parts[7]) {
                    return Ok(Some((heading_b, lat, lon)));
                }
            }
        }

        Ok(None)
    }

    /// Update all aircraft positions and states
    fn update_aircraft(&mut self, delta_time: f64) {
        let sim_config = self.sim_config.clone();
        let nav_db = self.nav_db.clone();

        // Collect callsigns of aircraft that will be removed
        let removed_callsigns: Vec<String> = self
            .aircraft
            .iter()
            .filter(|a| a.is_route_complete())
            .map(|a| a.callsign.clone())
            .collect();

        // Remove completed aircraft from used callsigns
        for callsign in &removed_callsigns {
            self.used_callsigns.remove(callsign);
            info!(
                "[SIMULATOR] Aircraft {} completed route and removed",
                callsign
            );
        }

        // Remove aircraft that have completed their routes
        self.aircraft.retain(|a| !a.is_route_complete());

        // Update remaining aircraft
        for aircraft in &mut self.aircraft {
            aircraft.update(delta_time, &nav_db, &sim_config);
        }
    }

    /// Create departure spawn timers
    fn create_departure_timers(&self) -> Vec<(String, u64, u64)> {
        self.scenario
            .departure_configs()
            .iter()
            .map(|dep| {
                let interval_ticks =
                    (dep.interval as f64 / (1.0 / self.sim_config.radar_update_rate)) as u64;
                (dep.departing.clone(), interval_ticks, 0u64)
            })
            .collect()
    }

    /// Create transit spawn timers
    fn create_transit_timers(&self) -> Vec<(usize, u64, u64)> {
        self.scenario
            .transit_configs()
            .iter()
            .enumerate()
            .map(|(idx, transit)| {
                let interval_ticks =
                    (transit.interval as f64 / (1.0 / self.sim_config.radar_update_rate)) as u64;
                (idx, interval_ticks, 0u64)
            })
            .collect()
    }

    /// Check and spawn departures
    async fn check_departure_spawns(
        &mut self,
        timers: &mut [(String, u64, u64)],
        loop_count: u64,
    ) -> Result<()> {
        for (aerodrome, interval, last_spawn) in timers.iter_mut() {
            if loop_count - *last_spawn >= *interval {
                *last_spawn = loop_count;

                if let Some(route) = self.scenario.random_departure_route(aerodrome) {
                    let departure = aerodrome.clone();
                    let arrival = route.arriving.clone();
                    let route_str = route.route.clone();
                    self.spawn_departure(&departure, &arrival, &route_str)
                        .await?;
                }
            }
        }
        Ok(())
    }

    /// Spawn a departure aircraft
    async fn spawn_departure(&mut self, departure: &str, arrival: &str, route: &str) -> Result<()> {
        // Get airport coordinates
        let airport_coords = self.get_airport_coords(departure)?;

        // Get runway information
        let runway = match self.scenario.active_runway(departure) {
            Some(r) => r.to_string(),
            None => return Err(anyhow::anyhow!("No active runway for {}", departure)),
        };

        // Parse runway heading (e.g., "27R" -> 270 degrees)
        let runway_heading = self.parse_runway_heading(&runway);

        // Generate callsign
        let callsign = self.generate_callsign(departure)?;

        // Select aircraft type
        let aircraft_type = self.select_aircraft_type(departure)?;

        // Assign squawk
        let squawk = self.assign_squawk();

        // Create aircraft
        let aircraft = Aircraft::new_departure(
            callsign.clone(),
            aircraft_type.clone(),
            squawk.clone(),
            departure.to_string(),
            arrival.to_string(),
            route.to_string(),
            self.get_cruise_altitude(route),
            runway,
            airport_coords,
            runway_heading,
        );

        let resolved_owner = self
            .resolve_ownership(&aircraft)
            .map(|decision| decision.owner_callsign);
        info!(
            "[SIMULATOR] Spawned departure {} ({}) from {} to {} via {}",
            callsign,
            aircraft.aircraft_type,
            departure,
            arrival,
            aircraft.current_fix().unwrap_or("route")
        );

        // Get flight plan before moving aircraft
        let flight_plan_str = aircraft.flight_plan.to_fsd_string();

        // Login pilot to FSD server and send flight plan
        self.login_pilot(&callsign, &aircraft_type, &squawk, &flight_plan_str)
            .await?;

        // Send initial position immediately after login
        if let Some(pilot) = self.pilot_clients.get_mut(&callsign) {
            pilot
                .send_position(
                    aircraft.latitude,
                    aircraft.longitude,
                    aircraft.altitude,
                    aircraft.ground_speed,
                    aircraft.heading,
                    &aircraft.squawk,
                )
                .await?;
        }

        // Mark callsign as used
        self.used_callsigns.insert(callsign.clone());

        // Publish assigned squawk from the resolved owner when that owner is AI.
        let bc_sender = resolved_owner
            .filter(|owner| self.is_ai_controller(owner))
            .or_else(|| {
                self.ai_controllers
                    .first()
                    .map(|controller| controller.callsign().to_string())
            });
        if let Some(sender) = bc_sender {
            let bc_message = format!("$CQ{}:@94835:BC:{}:{}", sender, callsign, squawk);
            if !self.send_from_ai_controller(&sender, &bc_message) {
                warn!("[SIMULATOR] Failed to queue BC message for {}", callsign);
            }
        }

        self.aircraft.push(aircraft);

        Ok(())
    }

    /// Login a pilot client to the FSD server
    async fn login_pilot(
        &mut self,
        callsign: &str,
        aircraft_type: &str,
        squawk: &str,
        flight_plan: &str,
    ) -> Result<()> {
        let mut pilot = AiPilot::new(callsign.to_string());
        pilot.connect(&self.server_addr).await?;
        pilot.login(aircraft_type, squawk).await?;

        // Send flight plan
        pilot.send_flight_plan(flight_plan).await?;

        self.pilot_clients.insert(callsign.to_string(), pilot);
        Ok(())
    }

    /// Broadcast all pilot positions to FSD server
    async fn broadcast_pilot_positions(&mut self) -> Result<()> {
        let mut disconnected = Vec::new();

        for aircraft in &self.aircraft {
            if let Some(pilot) = self.pilot_clients.get_mut(&aircraft.callsign) {
                if let Err(_e) = pilot
                    .send_position(
                        aircraft.latitude,
                        aircraft.longitude,
                        aircraft.altitude,
                        aircraft.ground_speed,
                        aircraft.heading,
                        &aircraft.squawk,
                    )
                    .await
                {
                    disconnected.push(aircraft.callsign.clone());
                }
            }
        }

        // Remove disconnected pilots
        for callsign in disconnected {
            self.pilot_clients.remove(&callsign);
        }

        Ok(())
    }

    /// Get airport coordinates from navigation database
    fn get_airport_coords(&self, icao: &str) -> Result<(f64, f64)> {
        // Try to find airport in fix database
        if let Some(coords) = self.nav_db.get(icao) {
            return Ok(*coords);
        }

        // Default coordinates for common UK airports
        let coords = match icao {
            "EGSS" => (51.885, 0.235),  // Stansted
            "EGGW" => (51.875, -0.368), // Luton
            "EGLC" => (51.505, 0.055),  // London City
            "EGLL" => (51.471, -0.461), // Heathrow
            "EGKK" => (51.148, -0.190), // Gatwick
            _ => return Err(anyhow::anyhow!("Unknown airport: {}", icao)),
        };

        Ok(coords)
    }

    /// Parse runway heading from runway identifier
    fn parse_runway_heading(&self, runway: &str) -> i32 {
        // Extract numeric part (e.g., "27R" -> 27)
        let numeric: String = runway.chars().filter(|c| c.is_numeric()).collect();
        if let Ok(rwy_num) = numeric.parse::<i32>() {
            rwy_num * 10
        } else {
            0
        }
    }

    /// Generate a unique callsign for an aircraft
    fn generate_callsign(&mut self, departure: &str) -> Result<String> {
        let mut rng = rand::thread_rng();

        // Get airline for this airport
        let airlines = self
            .fleet_config
            .airports
            .get(departure)
            .ok_or_else(|| anyhow::anyhow!("No airlines configured for {}", departure))?;

        // Try up to 100 times to generate a unique callsign
        for _ in 0..100 {
            let airline = airlines
                .get(rng.gen_range(0..airlines.len()))
                .ok_or_else(|| anyhow::anyhow!("No airline selected"))?;

            // Generate flight number
            let flight_num = rng.gen_range(1..9999);
            let callsign = format!("{}{:04}", airline, flight_num);

            // Check if callsign is unique
            if !self.used_callsigns.contains(&callsign) {
                return Ok(callsign);
            }
        }

        Err(anyhow::anyhow!(
            "Failed to generate unique callsign after 100 attempts"
        ))
    }

    /// Select an aircraft type for departure
    fn select_aircraft_type(&self, departure: &str) -> Result<String> {
        let mut rng = rand::thread_rng();

        // Get airlines for this airport
        let airlines = self
            .fleet_config
            .airports
            .get(departure)
            .ok_or_else(|| anyhow::anyhow!("No airlines for {}", departure))?;

        let airline = airlines
            .get(rng.gen_range(0..airlines.len()))
            .ok_or_else(|| anyhow::anyhow!("No airline selected"))?;

        // Get aircraft types for this airline
        let aircraft_types = self.fleet_config.airlines.get(airline);

        if aircraft_types.is_none() || aircraft_types.unwrap().is_empty() {
            warn!(
                "[SIMULATOR] No aircraft types configured for airline {}, using default A320",
                airline
            );
            return Ok("A320".to_string());
        }

        let aircraft_types = aircraft_types.unwrap();
        let aircraft_type = aircraft_types
            .get(rng.gen_range(0..aircraft_types.len()))
            .ok_or_else(|| anyhow::anyhow!("No aircraft type selected"))?;

        Ok(aircraft_type.clone())
    }

    /// Assign a squawk code
    fn assign_squawk(&mut self) -> String {
        if let Some(squawk) = self.squawk_pool.pop() {
            format!("{:04}", squawk)
        } else {
            // Fallback if pool is empty
            let mut rng = rand::thread_rng();
            format!("{:04}", rng.gen_range(2000..7777))
        }
    }

    /// Extract cruise altitude from route
    fn get_cruise_altitude(&self, route: &str) -> u32 {
        // Look for FL in route (e.g., FL350)
        if let Some(fl_pos) = route.find("FL") {
            let fl_str = &route[fl_pos + 2..];
            if let Some(num_end) = fl_str.find(|c: char| !c.is_numeric()) {
                if let Ok(fl) = fl_str[..num_end].parse::<u32>() {
                    return fl;
                }
            }
        }

        // Default cruise altitude
        360
    }

    /// Spawn an enroute transit aircraft
    async fn spawn_transit(
        &mut self,
        departure: &str,
        arrival: &str,
        route: &str,
        current_level_ft: u32,
        cruise_level_ft: u32,
        first_controller: &str,
    ) -> Result<()> {
        let callsign = self.generate_callsign(departure)?;
        let aircraft_type = self.select_aircraft_type(departure)?;
        let squawk = self.assign_squawk();

        // Reuse route parsing/flight-plan setup from departure construction,
        // then shift to an enroute state at a known fix.
        let mut aircraft = Aircraft::new_departure(
            callsign.clone(),
            aircraft_type.clone(),
            squawk.clone(),
            departure.to_string(),
            arrival.to_string(),
            route.to_string(),
            (cruise_level_ft / 100).max(1),
            "00".to_string(),
            (0.0, 0.0),
            0,
        );

        // For arrivals/transits, route strings often end with STAR names
        // (e.g. LOGAN2H) that are not direct fixes. Expand these into
        // waypoint sequences so aircraft do not terminate at the STAR entry.
        if let Some(last_token) = route.split_whitespace().last() {
            let star_candidate = last_token
                .split('/')
                .next()
                .unwrap_or(last_token)
                .trim()
                .to_uppercase();

            if star_candidate.chars().any(|c| c.is_ascii_alphabetic())
                && star_candidate.chars().any(|c| c.is_ascii_digit())
            {
                if let Some((star_fixes, active_runway)) =
                    self.resolve_star_fixes(arrival, &star_candidate)?
                {
                    let added = aircraft.append_route_fixes(star_fixes);
                    if added > 0 {
                        info!(
                            "[SIMULATOR] {} transit STAR {} appended {} fixes for runway {}",
                            callsign, star_candidate, added, active_runway
                        );
                    }
                }
            }
        }

        let Some(spawn_fix_index) = aircraft
            .route_fixes
            .iter()
            .position(|fix| self.nav_db.contains_key(fix))
        else {
            return Err(anyhow::anyhow!(
                "No known navigation fix available for transit route: {}",
                route
            ));
        };

        let spawn_fix = aircraft.route_fixes[spawn_fix_index].clone();
        let (spawn_lat, spawn_lon) = *self.nav_db.get(&spawn_fix).ok_or_else(|| {
            anyhow::anyhow!("Missing nav coordinates for transit fix {}", spawn_fix)
        })?;

        aircraft.latitude = spawn_lat;
        aircraft.longitude = spawn_lon;
        aircraft.current_fix_index =
            (spawn_fix_index + 1).min(aircraft.route_fixes.len().saturating_sub(1));

        if aircraft.current_fix_index < aircraft.route_fixes.len() {
            if let Some((next_lat, next_lon)) = self
                .nav_db
                .get(&aircraft.route_fixes[aircraft.current_fix_index])
            {
                let initial_heading = heading_from_to(spawn_lat, spawn_lon, *next_lat, *next_lon);
                aircraft.heading = initial_heading;
                aircraft.target_heading = initial_heading;
            }
        }

        aircraft.altitude = current_level_ft as i32;
        // Standard transits spawn level at inbound altitude; do not auto-climb.
        aircraft.target_altitude = current_level_ft as i32;
        aircraft.auto_climb_to_cruise = false;
        aircraft.ground_speed = if current_level_ft >= 20_000 {
            420
        } else if current_level_ft >= 10_000 {
            300
        } else {
            250
        };
        aircraft.target_speed = aircraft.ground_speed;
        aircraft.phase = FlightPhase::Cruise;

        let first_controller = first_controller.trim();
        let resolved_owner = self
            .resolve_ownership(&aircraft)
            .map(|decision| decision.owner_callsign);
        let default_handoff_fix = aircraft
            .route_fixes
            .get((spawn_fix_index + 1).min(aircraft.route_fixes.len().saturating_sub(1)))
            .cloned()
            .unwrap_or_else(|| spawn_fix.clone());
        let agreement = self.agreement_resolver.as_ref().and_then(|resolver| {
            resolver.resolve_internal_transit(departure, arrival, &aircraft.route_fixes)
        });
        let agreed_altitude_ft = agreement
            .as_ref()
            .and_then(|decision| decision.agreed_altitude_ft)
            .unwrap_or(current_level_ft as i32);
        let handoff_fix = agreement
            .as_ref()
            .and_then(|decision| decision.handoff_fix.clone())
            .filter(|fix| self.nav_db.contains_key(fix))
            .unwrap_or(default_handoff_fix);

        if let Some(decision) = agreement.as_ref() {
            let agreed_altitude_log = decision
                .agreed_altitude_ft
                .map(|altitude| format!("{}ft", altitude))
                .unwrap_or_else(|| "none".to_string());
            let handoff_fix_log = decision
                .handoff_fix
                .clone()
                .unwrap_or_else(|| "none".to_string());
            info!(
                "[SIMULATOR] Transit agreement {}: {} ({}) => level {}, handoff {}",
                callsign,
                decision.matched_rule,
                decision.matched_source,
                agreed_altitude_log,
                handoff_fix_log
            );
        } else {
            info!(
                "[SIMULATOR] Transit agreement {}: no match for {} -> {} via {}",
                callsign, departure, arrival, route
            );
        }

        if !first_controller.is_empty() && self.is_controller_online(first_controller) {
            aircraft.set_assumed_by(Some(first_controller.to_string()));
            self.send_assumed_tag_marker(first_controller, &callsign);
        } else {
            let fallback_ai_owner = self.select_transit_fallback_owner(resolved_owner.as_ref());
            if let Some(owner) = fallback_ai_owner {
                aircraft.set_assumed_by(Some(owner.clone()));
                self.send_assumed_tag_marker(&owner, &callsign);
                aircraft.assign_altitude(agreed_altitude_ft);
                if !first_controller.is_empty() {
                    self.pending_transit_handoffs.insert(
                        callsign.clone(),
                        PendingTransitProfileHandoff {
                            preferred_controller: first_controller.to_string(),
                            handoff_fix: handoff_fix.clone(),
                            agreed_altitude_ft: agreed_altitude_ft,
                        },
                    );
                    info!(
                        "[SIMULATOR] Transit {} fallback owner {} ({} offline), agreed {}ft at {}",
                        callsign, owner, first_controller, agreed_altitude_ft, handoff_fix
                    );
                }
            }
        }

        info!(
            "[SIMULATOR] Spawned transit {} ({}) {} -> {} at FL{:03} via {}",
            callsign,
            aircraft.aircraft_type,
            departure,
            arrival,
            current_level_ft / 100,
            aircraft.current_fix().unwrap_or("route")
        );

        let flight_plan_str = aircraft.flight_plan.to_fsd_string();
        self.login_pilot(&callsign, &aircraft_type, &squawk, &flight_plan_str)
            .await?;

        if let Some(pilot) = self.pilot_clients.get_mut(&callsign) {
            pilot
                .send_position(
                    aircraft.latitude,
                    aircraft.longitude,
                    aircraft.altitude,
                    aircraft.ground_speed,
                    aircraft.heading,
                    &aircraft.squawk,
                )
                .await?;
        }

        self.used_callsigns.insert(callsign.clone());

        let bc_sender = aircraft
            .assumed_by
            .as_ref()
            .filter(|owner| self.is_ai_controller(owner))
            .cloned()
            .or_else(|| resolved_owner.filter(|owner| self.is_ai_controller(owner)))
            .or_else(|| {
                self.ai_controllers
                    .first()
                    .map(|controller| controller.callsign().to_string())
            });
        if let Some(sender) = bc_sender {
            let bc_message = format!("$CQ{}:@94835:BC:{}:{}", sender, callsign, squawk);
            if !self.send_from_ai_controller(&sender, &bc_message) {
                warn!("[SIMULATOR] Failed to queue BC message for {}", callsign);
            }
        }

        self.aircraft.push(aircraft);
        Ok(())
    }

    /// Check and spawn transits
    async fn check_transit_spawns(
        &mut self,
        timers: &mut [(usize, u64, u64)],
        loop_count: u64,
    ) -> Result<()> {
        for (idx, interval, last_spawn) in timers.iter_mut() {
            if loop_count - *last_spawn >= *interval {
                *last_spawn = loop_count;

                if let Some(route) = self.scenario.random_transit_route(*idx) {
                    let departure = route.departing.clone();
                    let arrival = route.arriving.clone();
                    let route_str = route.route.clone();
                    let current_level = route.current_level;
                    let cruise_level = route.cruise_level;
                    let first_controller = route.first_controller.clone();
                    info!(
                        "[SIMULATOR] Spawning transit: {} -> {} at FL{:03} via {}",
                        departure,
                        arrival,
                        current_level / 100,
                        route_str
                    );
                    self.spawn_transit(
                        &departure,
                        &arrival,
                        &route_str,
                        current_level,
                        cruise_level,
                        &first_controller,
                    )
                    .await?;
                }
            }
        }
        Ok(())
    }

    /// Stop the simulation
    pub async fn stop(&mut self) -> Result<()> {
        info!("[SIMULATOR] Stopping simulation...");
        self.running = false;

        // Disconnect all pilots
        for (callsign, mut pilot) in self.pilot_clients.drain() {
            info!("[SIMULATOR] Disconnecting pilot {}", callsign);
            pilot.disconnect().await?;
        }

        // Disconnect all AI controllers
        for controller in &mut self.ai_controllers {
            controller.disconnect().await?;
        }

        self.ai_controllers.clear();

        info!("[SIMULATOR] Simulation stopped");
        Ok(())
    }

    /// Get simulation statistics
    pub fn statistics(&self) -> SimulatorStats {
        SimulatorStats {
            running: self.running,
            active_controllers: self.ai_controllers.len(),
            active_pilots: 0, // TODO: Track pilots
            scenario_name: self.scenario.name.clone(),
        }
    }
}

/// Statistics about the running simulator
#[derive(Debug, Clone)]
pub struct SimulatorStats {
    pub running: bool,
    pub active_controllers: usize,
    pub active_pilots: usize,
    pub scenario_name: String,
}

impl std::fmt::Display for SimulatorStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Simulator Status:")?;
        writeln!(f, "  Scenario: {}", self.scenario_name)?;
        writeln!(f, "  Running: {}", self.running)?;
        writeln!(f, "  Active Controllers: {}", self.active_controllers)?;
        writeln!(f, "  Active Pilots: {}", self.active_pilots)?;
        Ok(())
    }
}
