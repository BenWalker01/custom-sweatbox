# TODO

This checklist tracks features that exist in the legacy Python sweatbox but are still missing or only partially implemented in Rust.

## P0 - Core Training Control Gaps

- [x] Implement HOLD command behavior (racetrack logic, timed legs, fix-specific defaults)
- [x] Implement STAR assignment command behavior (append STAR fixes from airport STAR data)
- [ ] Implement LVL-by-fix behavior (be level by waypoint, not just direct-to parsing)
- [ ] Implement full handoff flow: outbound HO generation, inbound HA/HT behavior 
- [ ] Implement release-point ownership model (`currentlyWithData` equivalent) with automatic handoff triggers on route

## P1 - Aircraft Behavior 

- [ ] Replace simplified vertical model with aircraft-performance-table based climb/descent rates by altitude band
- [ ] Restore Python-like energy model coupling between acceleration and climb/descent performance
- [ ] Add speed policy  around 10,000 ft (below/above constraints and transitions)
- [ ] Add approach/VREF deceleration model per aircraft type
- [ ] Improve ILS to Python-level behavior (intercept geometry, glide capture edge cases, go-around conditions)

## P1 - Command/Protocol 

- [ ] Handle `$AM`-driven flightplan updates for STAR extraction/assignment
- [ ] Add  for `HO`/`HOAI` kill/cleanup behaviors tied to handoff rules
- [ ] Add  handling for legacy management packets (`WH`, `HT`, related flow-control packets)
- [ ] Add explicit logging/metrics for each accepted/rejected controller command reason

## P2 - Ground Operations 


## P2 - Traffic Generation/Profile 

- [ ] Implement transit spawning fully (currently logged as TODO in Rust)
- [ ] Implement overflight and before-fix spawn variants (Python `OVF`, `TR2` equivalents)
- [ ] Support profile options equivalent to Python behavior (`withMaster`, first-controller/top-down handling)
- [ ] Add top-down controller resolution equivalent to Python `findOnlineTopdownController`

## P3 - Operational Utilities 

- [ ] Add pause/resume/time-multiplier operational controls
- [ ] Add periodic save-state/snapshot support
- [ ] Add optional scenario/session restore from snapshot

## Notes

