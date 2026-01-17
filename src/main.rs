use anyhow::Result;
use clap::{Parser, Subcommand};
use tracing::{info, Level};
use std::sync::Arc;

mod server;
mod utils;
mod config;
mod scenario;
mod simulation;
mod aircraft;

use utils::navigation::load_navigation_data;
use utils::performance::load_performance_data;
use config::{SimulationConfig, FleetConfig};
use scenario::Scenario;
use simulation::Simulator;


#[derive(Parser)]
#[command(name = "custom-sweatbox")]
#[command(about = "Custom EuroScope aircraft control simulator", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    Server {
        #[arg(short, long, default_value = "6809")]
        port: u16,

        #[arg(short = 'H', long, default_value = "127.0.0.1")]
        host: String,
    },

    Simulator {
        #[arg(short, long, default_value = "127.0.0.1:6809")]
        server: String,

        #[arg(short, long)]
        profile: Option<String>,
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize tracing
    tracing_subscriber::fmt()
        .with_max_level(Level::INFO)
        .with_target(false)
        .init();

    let cli = Cli::parse();

    match cli.command {
        Commands::Server { port, host } => {
            info!("Starting FSD Server on {}:{}", host, port);
            let fsd_server = server::FsdServer::new(host, port);
            fsd_server.start().await?;
        }

        Commands::Simulator {
            server,
            profile,
        } => {
            info!("Starting Simulator connecting to {}", server);
            
            // Load navigation data
            info!("Loading navigation data...");
            let fix_db = match load_navigation_data("data") {
                Ok(db) => {
                    info!("Loaded {} fixes", db.len());
                    Arc::new(db)
                }
                Err(e) => {
                    eprintln!("Failed to load navigation data: {}", e);
                    return Err(e.into());
                }
            };
            
            // Load performance data
            info!("Loading aircraft performance data...");
            let perf_db = match load_performance_data("data/AircraftPerformace.txt") {
                Ok(db) => {
                    info!("Loaded performance data for {} aircraft types", db.len());
                    Arc::new(db)
                }
                Err(e) => {
                    eprintln!("Failed to load performance data: {}", e);
                    return Err(e.into());
                }
            };
            
            // Load profile
            let profile_path = profile.unwrap_or_else(|| "profiles/TCE + TCNE.json".to_string());
            info!("Loading simulation profile: {}", profile_path);
            
            // Load scenario using the new parser
            let scenario = Scenario::load(&profile_path)?;
            let stats = scenario.statistics();
            info!("{}", stats);

            // Create configuration
            let sim_config = SimulationConfig::default();
            let fleet_config = FleetConfig::default();

            // Create simulator
            let mut simulator = Simulator::new(
                scenario,
                sim_config,
                fleet_config,
                fix_db,
                perf_db,
                server,
            );

            // Initialize and run simulation
            info!("Initializing simulation...");
            simulator.initialize().await?;
            
            info!("Starting simulation...");
            
            // Create shutdown channel
            let (shutdown_tx, shutdown_rx) = tokio::sync::broadcast::channel(1);
            
            // Setup Ctrl+C handler
            ctrlc::set_handler(move || {
                info!("Received Ctrl+C, stopping simulation...");
                let _ = shutdown_tx.send(());
            }).expect("Error setting Ctrl-C handler");
            
            // Run simulation loop
            simulator.run(shutdown_rx).await?;
            
            // Stop simulation
            info!("Stopping simulation...");
            simulator.stop().await?;
            
            info!("Simulation stopped cleanly");
        }
    }

    Ok(())
}