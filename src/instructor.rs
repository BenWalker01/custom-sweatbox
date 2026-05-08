use anyhow::{Context, Result};
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::sync::{broadcast, mpsc};

use crate::config::{FleetConfig, SimulationConfig};
use crate::scenario::Scenario;
use crate::simulation::{Simulator, SimulatorControlCommand};
use crate::utils::navigation::load_navigation_data;
use crate::utils::performance::load_performance_data;

#[derive(Debug, Clone)]
struct SpawnGroupView {
    id: String,
    label: String,
    average_interval_seconds: f64,
    route_count: usize,
}

#[derive(Debug, Clone)]
struct SelectedScenario {
    path: PathBuf,
    name: String,
    spawn_groups: Vec<SpawnGroupView>,
}

struct RunningSimulation {
    scenario_name: String,
    speed_multiplier: f64,
    paused: bool,
    spawn_groups: Vec<SpawnGroupView>,
    shutdown_tx: broadcast::Sender<()>,
    control_tx: mpsc::UnboundedSender<SimulatorControlCommand>,
    join_handle: tokio::task::JoinHandle<Result<()>>,
}

pub async fn run_instructor_panel(server_addr: &str, profiles_dir: &str) -> Result<()> {
    let profile_paths = discover_profiles(profiles_dir)?;
    if profile_paths.is_empty() {
        println!("No profile JSON files found in {profiles_dir}");
        return Ok(());
    }

    let nav_db = Arc::new(load_navigation_data("data").context("Failed to load navigation data")?);
    let perf_db = Arc::new(
        load_performance_data("data/AircraftPerformace.txt")
            .context("Failed to load performance data")?,
    );

    let mut selected_index = 0usize;
    let mut selected = Some(load_selected_scenario(&profile_paths[selected_index])?);
    let mut running: Option<RunningSimulation> = None;

    print_help();
    print_scenarios(&profile_paths, selected_index);
    if let Some(current) = selected.as_ref() {
        println!("Selected scenario: {}", current.name);
        print_spawn_groups(&current.spawn_groups);
    }

    let stdin = BufReader::new(tokio::io::stdin());
    let mut lines = stdin.lines();

    loop {
        print!("instructor> ");
        io::stdout().flush()?;

        let Some(line) = lines.next_line().await? else {
            break;
        };
        let input = line.trim();
        if input.is_empty() {
            continue;
        }

        let mut parts = input.split_whitespace();
        let command = parts.next().unwrap_or_default().to_ascii_lowercase();

        match command.as_str() {
            "help" => print_help(),
            "scenarios" | "list" => {
                print_scenarios(&profile_paths, selected_index);
            }
            "select" => {
                let Some(index_text) = parts.next() else {
                    println!("Usage: select <index>");
                    continue;
                };
                let Ok(index_one_based) = index_text.parse::<usize>() else {
                    println!("Invalid index: {index_text}");
                    continue;
                };
                if index_one_based == 0 || index_one_based > profile_paths.len() {
                    println!("Index must be between 1 and {}", profile_paths.len());
                    continue;
                }
                selected_index = index_one_based - 1;
                selected = Some(load_selected_scenario(&profile_paths[selected_index])?);
                if let Some(current) = selected.as_ref() {
                    println!("Selected scenario: {}", current.name);
                    print_spawn_groups(&current.spawn_groups);
                }
            }
            "start" => {
                if running.is_some() {
                    println!("Simulation already running.");
                    continue;
                }
                let Some(current) = selected.clone() else {
                    println!("No scenario selected.");
                    continue;
                };
                let scenario = Scenario::load(&current.path)?;
                let mut simulator = Simulator::new(
                    scenario,
                    SimulationConfig::default(),
                    FleetConfig::default(),
                    nav_db.clone(),
                    perf_db.clone(),
                    server_addr.to_string(),
                );
                simulator.initialize().await?;

                let (shutdown_tx, shutdown_rx) = broadcast::channel(1);
                let (control_tx, control_rx) = mpsc::unbounded_channel();
                let join_handle = tokio::spawn(async move {
                    let run_result = simulator.run_with_controls(shutdown_rx, Some(control_rx)).await;
                    let stop_result = simulator.stop().await;
                    run_result.and(stop_result)
                });

                running = Some(RunningSimulation {
                    scenario_name: current.name.clone(),
                    speed_multiplier: 1.0,
                    paused: false,
                    spawn_groups: current.spawn_groups.clone(),
                    shutdown_tx,
                    control_tx,
                    join_handle,
                });

                println!("Started scenario: {}", current.name);
            }
            "stop" => {
                stop_running_simulation(&mut running).await;
            }
            "pause" => {
                if let Some(active) = running.as_mut() {
                    if active
                        .control_tx
                        .send(SimulatorControlCommand::Pause)
                        .is_err()
                    {
                        println!("Failed to send pause command (simulation not running).");
                        running = None;
                    } else {
                        active.paused = true;
                        println!("Simulation paused.");
                    }
                } else {
                    println!("Simulation is not running.");
                }
            }
            "play" | "resume" => {
                if let Some(active) = running.as_mut() {
                    if active
                        .control_tx
                        .send(SimulatorControlCommand::Resume)
                        .is_err()
                    {
                        println!("Failed to send resume command (simulation not running).");
                        running = None;
                    } else {
                        active.paused = false;
                        println!("Simulation resumed.");
                    }
                } else {
                    println!("Simulation is not running.");
                }
            }
            "speedup" => {
                if let Some(active) = running.as_mut() {
                    let new_speed = (active.speed_multiplier * 2.0).min(16.0);
                    if active
                        .control_tx
                        .send(SimulatorControlCommand::SetSpeedMultiplier(new_speed))
                        .is_err()
                    {
                        println!("Failed to send speed command (simulation not running).");
                        running = None;
                    } else {
                        active.speed_multiplier = new_speed;
                        println!("Speed set to {:.2}x", new_speed);
                    }
                } else {
                    println!("Simulation is not running.");
                }
            }
            "slowdown" => {
                if let Some(active) = running.as_mut() {
                    let new_speed = (active.speed_multiplier / 2.0).max(0.25);
                    if active
                        .control_tx
                        .send(SimulatorControlCommand::SetSpeedMultiplier(new_speed))
                        .is_err()
                    {
                        println!("Failed to send speed command (simulation not running).");
                        running = None;
                    } else {
                        active.speed_multiplier = new_speed;
                        println!("Speed set to {:.2}x", new_speed);
                    }
                } else {
                    println!("Simulation is not running.");
                }
            }
            "speed" => {
                let Some(speed_text) = parts.next() else {
                    println!("Usage: speed <multiplier>");
                    continue;
                };
                let Ok(multiplier) = speed_text.parse::<f64>() else {
                    println!("Invalid multiplier: {speed_text}");
                    continue;
                };
                if multiplier <= 0.0 {
                    println!("Multiplier must be greater than 0.");
                    continue;
                }
                if let Some(active) = running.as_mut() {
                    if active
                        .control_tx
                        .send(SimulatorControlCommand::SetSpeedMultiplier(multiplier))
                        .is_err()
                    {
                        println!("Failed to send speed command (simulation not running).");
                        running = None;
                    } else {
                        active.speed_multiplier = multiplier;
                        println!("Speed set to {:.2}x", multiplier);
                    }
                } else {
                    println!("Simulation is not running.");
                }
            }
            "groups" => {
                if let Some(active) = running.as_ref() {
                    print_spawn_groups(&active.spawn_groups);
                } else if let Some(current) = selected.as_ref() {
                    print_spawn_groups(&current.spawn_groups);
                } else {
                    println!("No scenario selected.");
                }
            }
            "set-group" => {
                let Some(group_id) = parts.next() else {
                    println!("Usage: set-group <group-id> <seconds>");
                    continue;
                };
                let Some(seconds_text) = parts.next() else {
                    println!("Usage: set-group <group-id> <seconds>");
                    continue;
                };
                let Ok(seconds) = seconds_text.parse::<f64>() else {
                    println!("Invalid seconds value: {seconds_text}");
                    continue;
                };
                if seconds <= 0.0 {
                    println!("Seconds must be greater than 0.");
                    continue;
                }

                if let Some(active) = running.as_mut() {
                    let normalized_group_id = group_id.trim().to_uppercase();
                    if let Some(group) = active
                        .spawn_groups
                        .iter_mut()
                        .find(|group| group.id == normalized_group_id)
                    {
                        group.average_interval_seconds = seconds;
                        if active
                            .control_tx
                            .send(SimulatorControlCommand::SetSpawnGroupInterval {
                                group_id: normalized_group_id.clone(),
                                interval_seconds: seconds,
                            })
                            .is_err()
                        {
                            println!("Failed to apply spawn group change (simulation not running).");
                            running = None;
                        } else {
                            println!(
                                "Updated {} to {:.1}s average spawn interval.",
                                normalized_group_id, seconds
                            );
                        }
                    } else {
                        println!("Unknown group id: {normalized_group_id}");
                        print_spawn_groups(&active.spawn_groups);
                    }
                } else {
                    println!("Simulation is not running. Start it before changing live intervals.");
                }
            }
            "status" => {
                if let Some(active) = running.as_ref() {
                    println!(
                        "Running: {} | paused: {} | speed: {:.2}x",
                        active.scenario_name, active.paused, active.speed_multiplier
                    );
                } else if let Some(current) = selected.as_ref() {
                    println!("Stopped | selected scenario: {}", current.name);
                } else {
                    println!("Stopped | no scenario selected");
                }
            }
            "quit" | "exit" => {
                stop_running_simulation(&mut running).await;
                break;
            }
            _ => {
                println!(
                    "Unknown command: {}. Use `help` to see available commands.",
                    command
                );
            }
        }
    }

    Ok(())
}

fn discover_profiles(profiles_dir: &str) -> Result<Vec<PathBuf>> {
    let mut profiles: Vec<PathBuf> = fs::read_dir(profiles_dir)
        .with_context(|| format!("Failed to read profiles directory: {profiles_dir}"))?
        .filter_map(|entry| entry.ok().map(|e| e.path()))
        .filter(|path| path.extension().is_some_and(|ext| ext.eq_ignore_ascii_case("json")))
        .collect();

    profiles.sort_unstable_by(|a, b| a.file_name().cmp(&b.file_name()));
    Ok(profiles)
}

fn load_selected_scenario(path: &Path) -> Result<SelectedScenario> {
    let scenario = Scenario::load(path)?;
    Ok(SelectedScenario {
        path: path.to_path_buf(),
        name: scenario.name.clone(),
        spawn_groups: build_spawn_groups(&scenario),
    })
}

fn build_spawn_groups(scenario: &Scenario) -> Vec<SpawnGroupView> {
    let mut groups = Vec::new();

    for departure in scenario.departure_configs() {
        groups.push(SpawnGroupView {
            id: format!("DEP:{}", departure.departing.to_uppercase()),
            label: format!("Departure {}", departure.departing),
            average_interval_seconds: departure.interval as f64,
            route_count: departure.routes.len(),
        });
    }

    for (index, transit) in scenario.transit_configs().iter().enumerate() {
        groups.push(SpawnGroupView {
            id: format!("TRN:{}", index),
            label: format!("Transit group {}", index),
            average_interval_seconds: transit.interval as f64,
            route_count: transit.routes.len(),
        });
    }

    groups
}

fn print_help() {
    println!("Instructor panel commands:");
    println!("  help                     Show this help");
    println!("  scenarios | list         List available scenarios");
    println!("  select <index>           Select scenario by 1-based index");
    println!("  start                    Start selected scenario");
    println!("  stop                     Stop running scenario");
    println!("  pause                    Pause simulation updates");
    println!("  play | resume            Resume simulation updates");
    println!("  speedup                  Double speed multiplier");
    println!("  slowdown                 Halve speed multiplier");
    println!("  speed <multiplier>       Set exact speed multiplier");
    println!("  groups                   Show spawn groups");
    println!("  set-group <id> <seconds> Update live spawn interval for group id");
    println!("  status                   Show current panel state");
    println!("  quit | exit              Stop and exit panel");
}

fn print_scenarios(profiles: &[PathBuf], selected_index: usize) {
    println!("Available scenarios:");
    for (index, profile) in profiles.iter().enumerate() {
        let marker = if index == selected_index { "*" } else { " " };
        let name = profile
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("<unknown>");
        println!("  {} {}. {}", marker, index + 1, name);
    }
}

fn print_spawn_groups(groups: &[SpawnGroupView]) {
    if groups.is_empty() {
        println!("No spawn groups in selected scenario.");
        return;
    }

    println!("Spawn groups:");
    for group in groups {
        println!(
            "  {:<10} {:<24} avg {:>6.1}s  routes {:>2}",
            group.id, group.label, group.average_interval_seconds, group.route_count
        );
    }
}

async fn stop_running_simulation(running: &mut Option<RunningSimulation>) {
    if let Some(active) = running.take() {
        let _ = active.control_tx.send(SimulatorControlCommand::Stop);
        let _ = active.shutdown_tx.send(());
        match active.join_handle.await {
            Ok(Ok(())) => {
                println!("Simulation stopped.");
            }
            Ok(Err(error)) => {
                println!("Simulation stopped with error: {error}");
            }
            Err(error) => {
                println!("Simulation task join error: {error}");
            }
        }
    } else {
        println!("Simulation is not running.");
    }
}
