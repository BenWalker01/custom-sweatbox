use anyhow::Result;
use std::sync::Arc;
use tracing::{info, debug};
use tokio::time::{interval, Duration};

use crate::scenario::Scenario;
use crate::config::{SimulationConfig, FleetConfig};
use crate::utils::navigation::FixDatabase;
use crate::utils::performance::PerformanceDatabase;
use super::ai_controller::AiController;

/// Main simulation controller
pub struct Simulator {
    scenario: Arc<Scenario>,
    sim_config: Arc<SimulationConfig>,
    fleet_config: Arc<FleetConfig>,
    nav_db: Arc<FixDatabase>,
    perf_db: Arc<PerformanceDatabase>,
    server_addr: String,
    ai_controllers: Vec<AiController>,
    running: bool,
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
            running: false,
        }
    }

    /// Initialize the simulation
    pub async fn initialize(&mut self) -> Result<()> {
        info!("[SIMULATOR] Initializing simulation...");
        
        // Display scenario information
        let stats = self.scenario.statistics();
        info!("{}", stats);
        
        // Login AI controllers
        self.login_ai_controllers().await?;
        
        info!("[SIMULATOR] Initialization complete");
        Ok(())
    }

    /// Login AI controllers to the FSD server
    async fn login_ai_controllers(&mut self) -> Result<()> {
        info!("[SIMULATOR] Logging in AI controllers...");
        
        let (master_callsign, master_freq) = self.scenario.master_controller();
        
        // Create and login master controller
        info!("[SIMULATOR] Creating master controller: {} on {}", master_callsign, master_freq);
        
        let mut master_controller = AiController::new(
            master_callsign.to_string(),
            master_freq.to_string(),
            51.5,  // Default latitude (central UK)
            -0.5,  // Default longitude
            300,   // Range in nautical miles
        );
        
        // Connect and login
        master_controller.connect(&self.server_addr).await?;
        master_controller.login().await?;
        
        // Wait a bit for the server to process
        tokio::time::sleep(Duration::from_millis(500)).await;
        
        // Send IP query
        master_controller.send_ip_query().await?;
        
        // Start message loop
        master_controller.start_message_loop().await?;
        
        self.ai_controllers.push(master_controller);
        
        info!("[SIMULATOR] Master controller {} logged in", master_callsign);
        
        // Login other controllers
        for (callsign, freq) in self.scenario.other_controllers() {
            info!("[SIMULATOR] Creating controller: {} on {}", callsign, freq);
            
            let mut controller = AiController::new(
                callsign.clone(),
                freq.clone(),
                51.5,
                -0.5,
                300,
            );
            
            controller.connect(&self.server_addr).await?;
            
            // Wait a bit between logins
            tokio::time::sleep(Duration::from_millis(200)).await;
            
            controller.login().await?;
            tokio::time::sleep(Duration::from_millis(300)).await;
            
            controller.send_ip_query().await?;
            controller.start_message_loop().await?;
            
            self.ai_controllers.push(controller);
            
            info!("[SIMULATOR] Controller {} logged in", callsign);
        }
        
        info!("[SIMULATOR] {} AI controllers logged in", self.ai_controllers.len());
        
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
                    
                    // Check departure timers
                    self.check_departure_spawns(&mut departure_timers, loop_count).await?;
                    
                    // Check transit timers
                    self.check_transit_spawns(&mut transit_timers, loop_count).await?;
                    
                    // Update all aircraft positions
                    if loop_count % 10 == 0 {
                        debug!("[SIMULATOR] Loop {}: {} controllers active", 
                               loop_count, self.ai_controllers.len());
                    }
                }
            }
        }
        
        self.running = false;
        info!("[SIMULATOR] Simulation loop stopped");
        Ok(())
    }

    /// Create departure spawn timers
    fn create_departure_timers(&self) -> Vec<(String, u64, u64)> {
        self.scenario.departure_configs()
            .iter()
            .map(|dep| {
                let interval_ticks = (dep.interval as f64 / (1.0 / self.sim_config.radar_update_rate)) as u64;
                (dep.departing.clone(), interval_ticks, 0u64)
            })
            .collect()
    }

    /// Create transit spawn timers
    fn create_transit_timers(&self) -> Vec<(usize, u64, u64)> {
        self.scenario.transit_configs()
            .iter()
            .enumerate()
            .map(|(idx, transit)| {
                let interval_ticks = (transit.interval as f64 / (1.0 / self.sim_config.radar_update_rate)) as u64;
                (idx, interval_ticks, 0u64)
            })
            .collect()
    }

    /// Check and spawn departures
    async fn check_departure_spawns(&self, timers: &mut [(String, u64, u64)], loop_count: u64) -> Result<()> {
        for (aerodrome, interval, last_spawn) in timers.iter_mut() {
            if loop_count - *last_spawn >= *interval {
                *last_spawn = loop_count;
                
                if let Some(route) = self.scenario.random_departure_route(aerodrome) {
                    info!("[SIMULATOR] Spawning departure: {} -> {} via {}", 
                          aerodrome, route.arriving, route.route);
                    // TODO: Create and spawn aircraft
                }
            }
        }
        Ok(())
    }

    /// Check and spawn transits
    async fn check_transit_spawns(&self, timers: &mut [(usize, u64, u64)], loop_count: u64) -> Result<()> {
        for (idx, interval, last_spawn) in timers.iter_mut() {
            if loop_count - *last_spawn >= *interval {
                *last_spawn = loop_count;
                
                if let Some(route) = self.scenario.random_transit_route(*idx) {
                    info!("[SIMULATOR] Spawning transit: {} -> {} at FL{:03} via {}", 
                          route.departing, route.arriving, route.current_level / 100, route.route);
                    // TODO: Create and spawn aircraft
                }
            }
        }
        Ok(())
    }

    /// Stop the simulation
    pub async fn stop(&mut self) -> Result<()> {
        info!("[SIMULATOR] Stopping simulation...");
        self.running = false;
        
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
