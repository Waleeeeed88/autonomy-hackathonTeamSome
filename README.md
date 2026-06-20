# Red Team Hack Sim — How to Play

You're piloting a drone through a branching course. At each junction a **clue**
tells you which way to turn. Make the right two turns, reach the correct target, and
"deliver" — that proves you read the clues and flew the right path.

**There are no hints in the world.** This page is your briefing — read it first.

---

## The course — two turns, then the target

### 1️⃣ Arrows — your first turn
Two arrows: one **red**, one **green** (randomised every run).
> **Go the way the GREEN arrow points.** Green = go, Red = wrong way.

### 2️⃣ Blue spheres — your second turn
After the first turn you'll reach a cluster of **1–5 blue spheres**. Count them:
> **Even (2, 4) → go Left.  Odd (1, 3, 5) → go Right.**

### 3️⃣ The four vehicles — the target
Your two turns lead to a room with **four vehicles**. Reach the **one** that matches your
path (**memorise this legend**):

| Your two turns | Vehicle | How to reach it |
|----------------|---------|-----------------|
| Left, Left   | **Tank**            | fly into it |
| Left, Right  | **Boat**            | fly into it |
| Right, Left  | **Jet**             | fly into it |
| Right, Right | **Ice-Cream Truck** | **land beside it** to deliver supplies |

---

## Winning

You win **only** if you flew the **correct route** (green arrow → sphere count) **and**
reached the matching vehicle. The result shows on screen:

- ✅ **MISSION PASSED** — right path, right target.
- ❌ **FAILED** — wrong vehicle, or the right type of vehicle in the wrong room.

You're on the clock — but accuracy beats speed: a wrong turn sends you to a room full of decoys.

---

## Tips

- **Every room has all four vehicles.** Three are traps; only the one matching *your*
  path counts. Know the legend cold before you start.
- **The ice-cream truck must be *landed* beside, not crashed into** — set down on the ground within a 50m radius.

---

## Starting the game

It's a standalone build — no install, just run it from this folder. Pass the **map** as the
first argument and any options after it. The options are the same on Windows and Linux.

### Windows
```powershell
# PowerShell — the --% makes the quotes pass through untouched
.\Red_Team_Hack_Sim.exe --% RedRoad -windowed -ResX=1280 -ResY=720
```
```cmd
:: cmd.exe — no --% needed
Red_Team_Hack_Sim.exe RedRoad -windowed -ResX=1280 -ResY=720
```

### Linux
```bash
./Red_Team_Hack_Sim.sh RedRoad -windowed -ResX=1280 -ResY=720
```

### Useful options

| Option | Effect |
|--------|--------|
| `RedRoad` · `RedRoad2` | which level to play (first argument; default is `RedRoad`) |
| `-windowed -ResX=1280 -ResY=720` | run in a window of that size |
| `-fullscreen` | run full screen |
| `-ExecCmds="Scalability 0"` | graphics quality — **0** Low · 1 Med · 2 High · 3 Epic |
| `-LiteMode` | low-GPU mode (ray-tracing & Lumen off, capped frame rate) |
| `-ini:Input:[/Script/Engine.InputSettings]:bCaptureMouseOnLaunch=False,[/Script/Engine.InputSettings]:DefaultViewportMouseCaptureMode=NoCapture,[/Script/Engine.InputSettings]:DefaultViewportMouseLockMode=DoNotLock` | prevent mouse lock on window focus |

---

## Resetting / retrying a run

Returning to the **start pad** restarts the round. From code:
`drone.set_pose(start_pose, reset_kinematics=True)` (teleport to spawn, velocity zeroed).
This reset is the **one sanctioned use of `set_pose`**;
teleporting anywhere else is off-limits (see HARD mode below).

On reset the round starts **fresh**: the clock zeroes **and the puzzle re-randomises** — new
arrow colours, a new sphere count, a new correct target. Every attempt is a new puzzle, so
there's no single answer to memorise.

---

## Flying by code — the API

You pilot the drone from a **Python script**. While the game is running it hosts a sim server
on ports **8989 / 8990**; connect with the ProjectAirSim Python client:

```python
from projectairsim import ProjectAirSimClient, World, Drone

client = ProjectAirSimClient(address="127.0.0.1", port_topics=8989, port_services=8990)
client.connect()
world = World(client, scene_config_name="sf_scene.jsonc", sim_config_path="sim_config/")
drone = Drone(client, world, "Drone1")
```

> Positions are in **NED metres**: X = north, Y = east, Z = **down** (up is negative). The
> drone spawns at the start line, which is the local origin.

### The config files (`sim_config/`)

`World(...)` loads two configs from `sim_config/` — **use these exact files**; `Drone1`, the
start-line origin, and the camera ids are what the course and the `RaceManager` expect.

- **`sf_scene.jsonc`** — spawns one drone, **`Drone1`**, at the **start line** (NED origin
  `0, -35, -0.1`) into whatever level is loaded; real-time clock. Pass it as
  `scene_config_name="sf_scene.jsonc"`.
- **`sf_robot.jsonc`** — the drone: quad-X 5-inch, `fast-physics`, **`simple-flight-api`**
  controller (the built-in autopilot behind `takeoff` / `hover` / `move_to…`). Its sensors
  (the **id** is what you pass to the getters):

  | id | type | read with |
  |----|------|-----------|
  | `FPV` | camera | `get_images("FPV", [0])` — nose cam, 640×480, 120° FOV (your recon view) |
  | `Chase` | camera | `get_images("Chase", [0])` — follow cam, 1280×720, 90° FOV |
  | `IMU1` | imu | `get_imu_data("IMU1")` |
  | `Barometer1` | barometer | `get_barometer_data("Barometer1")` |
  | `Magnetometer1` | magnetometer | `get_magnetometer_data("Magnetometer1")` |
  | `GPS1` | gps | `get_gps_data("GPS1")` — **EASY only** |
  | `Battery` | battery | `get_battery_state("Battery")` |

**Commanding the drone:**

| Call | Does |
|------|------|
| `drone.enable_api_control()` · `drone.arm()` | take control · arm motors |
| `await drone.takeoff_async()` · `land_async()` | take off · land |
| `await drone.hover_async()` | hold position |
| `await drone.move_to_position_async(n, e, d, velocity)` | fly to a local point |
| `await drone.move_on_path_async([[n,e,d], …], velocity)` | fly through a list of waypoints |
| `await drone.move_by_velocity_async(vn, ve, vd, duration)` | fly at a world-frame velocity |
| `await drone.move_by_velocity_body_frame_async(vx, vy, vz, duration)` | velocity in the drone's own frame |
| `await drone.rotate_to_yaw_async(yaw)` · `rotate_by_yaw_rate_async(rate, duration)` | turn to a heading · spin |
| `drone.set_controls(roll_rate, pitch_rate, yaw_rate, throttle)` | low-level / acro |
| `drone.cancel_last_task()` | interrupt the current move command |
| `drone.get_landed_state()` | on the ground, or flying? |

**Reading the drone (telemetry):**

| Call | Gives |
|------|-------|
| `drone.get_estimated_kinematics()` | onboard estimate of position / velocity / attitude (the controller's fused state) |
| `drone.get_imu_data("IMU1")` | accelerometer + gyro |
| `drone.get_barometer_data("Barometer1")` | pressure / altitude |
| `drone.get_magnetometer_data("Magnetometer1")` | heading reference |
| `drone.get_battery_state("Battery")` | battery level |
| `drone.get_gps_data("GPS1")` | global position — **EASY only** |
| `drone.get_images("FPV", [0])` | nose-camera frames — how you **see** the arrows & spheres (or `"Chase"`) |

> Pass the sensor **id** (the table under *The config files*), not the type — e.g. `"IMU1"`.

**Did I win?** Poll the race manager:

```python
import time

def wait_for_result(world):
    while True:
        s = world.get_object_float_property("RaceManager", "MissionState")  # 0 Idle 1 Running 2 Passed 3 Failed
        if s in (2, 3):
            t = world.get_object_float_property("RaceManager", "ElapsedSeconds")
            return ("PASSED" if s == 2 else "FAILED"), t
        time.sleep(0.2)
```

---

## EASY mode vs HARD mode

Same course — the difference is **which APIs you may use**. The level shows its mode as
`LEVEL: EASY` / `LEVEL: HARD` on the result popup. It's on your honour.

**🟢 EASY — anything goes.** The full API, including the "where am I, exactly?" oracles:
ground-truth position (`get_ground_truth_kinematics()`, `get_ground_truth_pose()`), GPS
(`get_gps_data()`, `get_ground_truth_geo_location()`, `get_estimated_geo_location()`), and GPS
waypoints (`move_to_geo_position_async()`, `move_on_geo_path_async()`).

**🔴 HARD — fly like a real drone.** Those oracles are **off-limits**; navigate with onboard
estimation, sensors, and the camera only:

| Off-limits in HARD | Why |
|--------------------|-----|
| `get_ground_truth_kinematics` · `get_ground_truth_pose` | perfect position = trivial navigation |
| `get_gps_data` · `get_ground_truth_geo_location` · `get_estimated_geo_location` | GPS hands you global position |
| `move_to_geo_position_async` · `move_on_geo_path_async` | GPS-waypoint autopilot |
| `set_pose` · `set_ground_truth_kinematics` | teleporting — banned in **both** modes (the **reset** is the only exception) |

Still allowed in HARD: every command above (including `move_to_position_async` — the autopilot
tracks the setpoint from its *own* estimate, exactly like real life), `get_estimated_kinematics`,
the onboard sensors (`IMU1`, `Barometer1`, `Magnetometer1`, `Battery`), and the **camera**
(`get_images`). Read the clues with the camera and fly by estimate.

## Troubleshooting

- there is a common crash that can occur when doing certain things while the connection to the sim is live, it is better to close the active connection python first if you experience game crash/stall.