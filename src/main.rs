use anyhow::Result;
use clap::{Parser, Subcommand};
use tracing::{info, Level};


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
            info!("Starting FSD Server");
        }

        Commands::Simulator {
            server,
            profile,
        } => {
            info!("Starting Simulator");
        }
    }

    Ok(())
}
