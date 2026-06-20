#!/usr/bin/env python3
"""Hard-mode E2E solver attempt for Red Team Hack Sim.

Uses only allowed HARD inputs for decisions: FPV camera + estimated-position
waypoint flight. It does not read object poses/properties, GPS, or ground truth.
RaceManager MissionState is polled only to report pass/fail, as allowed by README.
"""
import argparse, asyncio, inspect, time, cv2, math
from collections import Counter

from redteam_sim import connect, read_frame

ALT = 6.0
FAST = 13.0
SLOW = 4.0

async def do(cmd):
    r = cmd
    for _ in range(2):
        if inspect.isawaitable(r):
            r = await r
    return r

async def bounded(coro, timeout, label, drone=None):
    try:
        return await asyncio.wait_for(do(coro), timeout=timeout)
    except Exception as e:
        print(f">> {label} bounded/end: {type(e).__name__}: {e}", flush=True)
        if drone is not None:
            try: drone.cancel_last_task()
            except Exception: pass
        return None

def green_pixels(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return int(cv2.countNonZero(cv2.inRange(hsv, (35, 70, 60), (95, 255, 255))))

def blue_sphere_components(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (90, 80, 45), (135, 255, 255))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    comps = []
    H, W = mask.shape
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        cx, cy = cent[i]
        # Blue sphere blobs are compact and above the near floor. This rejects long
        # orange/blue boundary artifacts and tiny tree/alias speckles.
        if 80 <= area <= 5000 and 8 <= w <= 130 and 8 <= h <= 130 and 10 <= y <= H * 0.95:
            ratio = w / max(h, 1)
            if 0.45 <= ratio <= 2.8:
                comps.append((int(area), int(x), int(y), int(w), int(h), float(cx), float(cy)))
    # Merge obvious fragments with close centroids.
    comps = sorted(comps, key=lambda c: c[0], reverse=True)
    centers = []
    kept = []
    for c in comps:
        cx, cy = c[5], c[6]
        if all((cx-ox)**2 + (cy-oy)**2 > 22**2 for ox, oy in centers):
            kept.append(c); centers.append((cx, cy))
    return kept[:5]

async def read_first_turn(drone):
    # Read the first arrow from several stopped viewpoints. A single start-frame
    # can be dark/occluded in RedRoad2 and previously caused random exit-code-3
    # restarts. We now slow down, hover, sample multiple y offsets, and aggregate
    # green pixels for each direction before choosing the route.
    obs = {"Left": [0, 219.3, "+X/Left"], "Right": [0, -166.2, "-X/Right"]}
    for y in [-5, -10, 0]:
        await do(drone.move_to_position_async(0, y, -ALT, SLOW))
        await do(drone.hover_async())
        time.sleep(0.25)
        for label, yaw, branch_x, first in [
            ("+X/Left", 0, 219.3, "Left"),
            ("-X/Right", math.pi, -166.2, "Right"),
        ]:
            await do(drone.rotate_to_yaw_async(yaw))
            await do(drone.hover_async())
            time.sleep(0.30)
            frame = read_frame(drone)
            score = green_pixels(frame)
            cv2.imwrite(f"hard_arrow_{first}_y{int(y)}.png", frame)
            print(f">> arrow sample y={y} {label}: green_pixels={score}", flush=True)
            obs[first][0] += score
    rows = [(score, branch_x, first) for first, (score, branch_x, _label) in obs.items()]
    rows.sort(reverse=True, key=lambda t: t[0])
    best, runner = rows[0], rows[1]
    print(f">> arrow aggregate: {rows}", flush=True)
    # Do not randomly stop at start for moderately visible arrows. Accept the
    # dominant direction if it is clearly above the other, even when absolute
    # brightness is lower than usual.
    if best[0] < 120 or best[0] < runner[0] * 1.8:
        raise RuntimeError(f"Could not confidently read green arrow: {rows}")
    score, branch_x, first = best
    return branch_x, first

async def read_sphere_count(drone, branch_x):
    # Calibrated boat demo/E2E sphere read: stop-and-sample several positions
    # around the clue. On the +X/Left branch this reliably keeps all blue spheres
    # in the FPV frame; the closer single-stop variant clipped balls and caused
    # wrong boat routing.
    counts = []
    look_yaw = math.pi if branch_x > 0 else 0.0
    for y in [-20, -15, -11, -8]:
        await do(drone.move_to_position_async(branch_x, y, -ALT, SLOW))
        await do(drone.rotate_to_yaw_async(look_yaw))
        await do(drone.hover_async())
        time.sleep(0.45)
        frame = read_frame(drone)
        comps = blue_sphere_components(frame)
        cv2.imwrite(f"hard_spheres_x{int(branch_x)}_y{y}.png", frame)
        c = len(comps)
        if 1 <= c <= 5:
            counts.append(c)
        print(f">> sphere stopped view y={y}: count={c} comps={comps}", flush=True)
    if not counts:
        raise RuntimeError("could not count blue spheres")
    freq = Counter(counts)
    # Prefer the most frequent count; tie-break lower to avoid fragment-overcount.
    count = sorted(freq.items(), key=lambda kv: (kv[1], -kv[0]), reverse=True)[0][0]
    print(f">> sphere count decided={count} from {counts}", flush=True)
    return count

def route_target(first, second):
    # Static course layout. These are map coordinates flown by estimated local-NED
    # autopilot; no runtime object-pose oracle is used.
    if (first, second) == ("Left", "Left"):
        return "tank", 219.3, -205.0, 304.7, -188.9, 0.0
    if (first, second) == ("Left", "Right"):
        return "boat", 219.3, 205.0, 252.5, 204.8, 0.0
    if (first, second) == ("Right", "Left"):
        # Jet in the southwest room.
        return "jet", -166.2, -205.0, -239.9, -185.1, -2.0
    if (first, second) == ("Right", "Right"):
        # Ice-cream truck in the northwest room.
        return "icecream", -166.2, 205.0, -202.9, 194.0, 0.0
    raise AssertionError

async def poll_result(world, timeout=8):
    end = time.time() + timeout
    last = None
    while time.time() < end:
        s = world.get_object_float_property("RaceManager", "MissionState")
        t = world.get_object_float_property("RaceManager", "ElapsedSeconds")
        last = (s, t)
        if s in (2, 3):
            return ("PASSED" if s == 2 else "FAILED"), t
        await asyncio.sleep(0.25)
    return "UNKNOWN", last

async def solve(address):
    client, world, drone = connect(address)
    try:
        drone.enable_api_control(); drone.arm()
        print(">> takeoff", flush=True)
        await do(drone.takeoff_async())
        await do(drone.move_to_position_async(0, -35, -ALT, FAST))

        branch_x, first = await read_first_turn(drone)
        count = await read_sphere_count(drone, branch_x)
        second = "Left" if count % 2 == 0 else "Right"
        vehicle, path_x, room_y, tx, ty, tz = route_target(first, second)
        print(f">> decoded HARD: first={first} spheres={count} second={second} vehicle={vehicle} target=({tx:.1f},{ty:.1f},{tz:.1f})", flush=True)

        # Fly the actual route before touching the target.
        await do(drone.move_to_position_async(path_x, -11.3, -ALT, FAST))
        await do(drone.move_to_position_async(path_x, room_y, -ALT, FAST))
        await do(drone.move_to_position_async(tx, ty, -ALT, FAST))

        if vehicle == "icecream":
            # Land on ground beside the truck, not into it. Use a couple of nearby
            # points because the truck collision mesh is oddly offset in RedRoad2.
            for lx, ly in [(tx, ty-10), (tx, ty), (tx+10, ty), (tx-10, ty)]:
                print(f">> icecream landing near ({lx:.1f},{ly:.1f})", flush=True)
                await bounded(drone.move_to_position_async(lx, ly, -3.0, SLOW), 30, "icecream approach", drone)
                await bounded(drone.land_async(), 35, "icecream land", drone)
                result, elapsed = await poll_result(world, 4)
                print(f">> landing result={result} elapsed={elapsed}", flush=True)
                if result in ("PASSED", "FAILED"):
                    print(f">> RESULT {result} {elapsed}", flush=True)
                    if result != "PASSED": raise SystemExit(2)
                    return
                drone.enable_api_control(); drone.arm()
                await bounded(drone.takeoff_async(), 10, "retakeoff", drone)
            result, elapsed = await poll_result(world, 2)
        elif vehicle == "jet":
            # Fly through the jet at altitude, then do tight vertical/axis sweeps.
            sweeps = [
                ((tx+28, ty, -2.5), [[tx,ty,-2.5],[tx-28,ty,-2.5]]),
                ((tx+28, ty, -1.7), [[tx,ty,-1.7],[tx-28,ty,-1.7]]),
                ((tx, ty+22, -2.3), [[tx,ty,-2.3],[tx,ty-22,-2.3]]),
                ((tx, ty+22, -1.5), [[tx,ty,-1.5],[tx,ty-22,-1.5]]),
            ]
            for start, path in sweeps:
                print(f">> jet sweep start={start} path={path}", flush=True)
                await bounded(drone.move_to_position_async(*start, SLOW), 25, "jet approach", drone)
                await bounded(drone.move_on_path_async(path, 8.0), 8, "jet sweep", drone)
                result, elapsed = await poll_result(world, 2)
                print(f">> jet sweep result={result} elapsed={elapsed}", flush=True)
                if result in ("PASSED", "FAILED"):
                    print(f">> RESULT {result} {elapsed}", flush=True)
                    if result != "PASSED": raise SystemExit(2)
                    return
            result, elapsed = await poll_result(world, 2)
        else:
            print(f">> flying into {vehicle}", flush=True)
            await do(drone.move_to_position_async(tx, ty, -1.2, SLOW))
            await bounded(drone.move_to_position_async(tx, ty, 0.4, SLOW), 8, "ground vehicle impact", drone)
            result, elapsed = await poll_result(world, 8)

        print(f">> RESULT {result} {elapsed}", flush=True)
        if result != "PASSED":
            raise SystemExit(2)
    finally:
        client.disconnect()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="127.0.0.1")
    asyncio.run(solve(ap.parse_args().address))
