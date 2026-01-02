use anyhow::Result;
use clap::{Parser, Subcommand};
use tracing::{info, Level};
use std::sync::Arc;

mod server;
mod simulator;
mod utils;

use utils::navigation::load_navigation_data;
use utils::performance::load_performance_data;


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
            let fsd_server = server::FsdServer::new();
            fsd_server.start().await?;
        }

        Commands::Simulator {
            server,
            profile: _,
        } => {
            info!("Starting Simulator connecting to {}", server);
            
            // Load navigation data
            info!("Loading navigation data...");
            let fix_db = match load_navigation_data("_old/data") {
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
            let perf_db = match load_performance_data("_old/data/AircraftPerformace.txt") {
                Ok(db) => {
                    info!("Loaded performance data for {} aircraft types", db.len());
                    Arc::new(db)
                }
                Err(e) => {
                    eprintln!("Failed to load performance data: {}", e);
                    return Err(e.into());
                }
            };
            
            // TODO: Implement simulator runner
            info!("Databases loaded successfully");
        }
    }

    Ok(())
}
