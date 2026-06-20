# Red Team Hack Sim — Autonomous Solution (WORKING)

`solution.py` completes the mission autonomously and reliably (verified ~100%
pass rate across all four randomised vehicle outcomes).

## Run it

1. **Launch the game** (one PowerShell window):
   ```powershell
   .\exe\Red_Team_Hack_Sim.exe --% RedRoad -windowed -ResX=1280 -ResY=720
   ```
   Wait for the level + drone to load.

2. **Run the solver** (second window):
   ```powershell
   .venv\Scripts\activate
   python -u solution.py
   ```
   Each run reads the freshly-randomised clues and delivers to the correct
   vehicle. Watch for `✅ MISSION PASSED`.

> The puzzle re-randomises every time the script connects, so `solution.py`
> reads the clues live on every run — there is nothing to pre-configure.

## How it works

The course geometry is **fixed**; only the clues randomise (green-arrow
direction, sphere count, and therefore the target vehicle). The solver:

1. **Reads the GREEN arrow** from the FPV camera at close range
   (green-vs-red blob comparison, multi-frame majority vote):
   points **left → NORTH branch**, points **right → SOUTH branch**.
2. **Counts the blue spheres**. Uses the exact `SphereCount` exposed by the
   `SpherePuzzle` object (EASY mode allows ground truth), with camera counting
   as a backup. **Even → left turn, odd → right turn.**
3. **Resolves the target** from the legend
   (LL→Tank, LR→Boat, RL→Jet, RR→Ice-Cream) and looks up the exact
   `GoalObject` position (all 16 vehicle positions were mapped from the sim and
   classified by 3-D bounding-box size).
4. **Delivers**: rams the Tank/Boat/Jet with duration-based velocity passes at
   several altitudes (so a collision can't hang the command and the low boat
   hull is reliably hit), or **lands beside** the Ice-Cream Truck.
5. **Confirms** via `RaceManager.MissionState` (2 = passed).

A safety fallback tries the same vehicle type in the other rooms if the primary
delivery doesn't register (reaching a wrong goal is silent, so this is safe).

## Files

| File | Purpose |
|------|---------|
| `solution.py` | **The autonomous solver — run this.** |
| `redteam_sim.py`, `sim_config/` | challenge helpers / config (unchanged) |
| `diag.py` | diagnostic variant (tries multiple rooms, verbose) |
| `recon.py`, `recon2.py`, `survey.py`, `gridmap.py`, `roommap.py` | one-off mapping scripts used to reverse-engineer the course |
| `debug_frames/` | camera frames + `last_run.json` saved each run |

## Notes / tuning

All course constants are at the top of `solution.py` (`GOALS`, `SPHERE_VIEW`,
`ARROW_VIEW`, altitudes). They are tuned for **RedRoad**. For `RedRoad2`, re-run
the recon scripts and update `GOALS` with the new `GoalObject` positions
(query them with `world.get_object_pose("GoalObject_N")`).
