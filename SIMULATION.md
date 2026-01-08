# Custom Sweatbox - Simulation Usage

## Overview

The simulation module enables automated aircraft spawning and control using FSD (Flight Sim Data) protocol. The system logs in AI controllers and spawns aircraft according to scenario profiles.

## Running the Simulation

### Start the FSD Server

```bash
cargo run --release -- server --port 6809 --host 127.0.0.1
```

### Start the Simulator

In a separate terminal:

```bash
cargo run --release -- simulator --server 127.0.0.1:6809 --profile "profiles/TCE + TCNE.json"
```

Or use the convenience script:

```bash
./run_simulation.fish
```

## What Happens

1. **Server Startup**: The FSD server starts listening for connections
2. **Scenario Loading**: The simulator loads the scenario profile (TCE + TCNE.json)
3. **AI Controller Login**: 
   - Master controller (LON_E_CTR on 18480) logs in
   - Additional controllers log in sequentially
4. **Main Loop**: The simulator runs at radar update rate (5 Hz)
   - Departure aircraft spawn at configured intervals
   - Transit aircraft spawn at configured intervals
   - Aircraft positions update each tick

## Scenario File Format

The scenario file (`profiles/TCE + TCNE.json`) defines:

- **Active Aerodromes**: Which airports are operational (EGSS, EGGW, EGLC, EGLL)
- **Active Runways**: Which runways are in use at each airport
- **Controllers**: Master and other AI controllers with their frequencies
- **Standard Departures**: 
  - Departing airport
  - Spawn interval (seconds)
  - Routes with destinations
- **Standard Transits**:
  - Spawn interval
  - Routes with origin, destination, altitude, and route string
  - First controller to handle the aircraft

## Example Scenario

```json
{
    "activeAerodromes": ["EGSS", "EGGW", "EGLC", "EGLL"],
    "activeRunways": {
        "EGSS": "22",
        "EGGW": "25"
    },
    "masterController": "LON_E_CTR",
    "masterControllerFreq": "18480",
    "stdDepartures": [
        {
            "departing": "EGSS",
            "interval": 180,
            "routes": [
                {"route": "CLN2E/22 CLN P44 RATLO M197 REDFA", "arriving": "EHAM"}
            ]
        }
    ]
}
```

## Current Implementation Status

✅ **Completed:**
- Scenario file parsing
- AI controller login to FSD server
- Main simulation loop
- Departure and transit spawn timers

🚧 **TODO:**
- Aircraft spawning and position updates
- Route parsing and navigation
- Flight plan filing
- Squawk assignment

## Architecture

```
Simulator
├── Scenario (parsed from JSON)
├── AI Controllers (connected to FSD)
│   ├── Master Controller
│   └── Other Controllers
└── Aircraft (to be implemented)
    ├── Departures
    └── Transits
```

## Shutdown

Press `Ctrl+C` to gracefully stop the simulation. The simulator will:
1. Stop the main loop
2. Disconnect all AI controllers
3. Clean up resources
