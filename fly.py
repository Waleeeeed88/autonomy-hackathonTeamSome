#!/usr/bin/env python3
"""Fast EASY-mode solver for Red Team Hack Sim latest build.

Uses FPV camera for the green arrow and exposed SpherePuzzle.SphereCount for
sphere parity, then flies the legal route and delivers to the matching vehicle.
"""
import argparse, asyncio, inspect, math, time
from dataclasses import dataclass

import cv2

from redteam_sim import connect, read_frame

ALT = 6.0
FAST = 12.0
SLOW = 4.0

async def do(cmd):
    r = cmd
    for _ in range(2):
        if inspect.isawaitable(r):
            r = await r
    return r

def green_pixels(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Latest build: green arrow is bright/saturated; orange/red guide arrows excluded.
    return int(cv2.countNonZero(cv2.inRange(hsv, (35, 70, 60), (95, 255, 255))))

async def first_turn_from_camera(drone):
    # At the junction, yaw 0 looks +X and yaw pi looks -X. The latest build's
    # floor arrow is most visible when looking along its travel direction.
    await do(drone.move_to_position_async(0, -5, -ALT, FAST))
    obs = []
    for label, yaw, branch_x, turn in [
        ("look_plus_x", 0.0, 106.3, "Left"),
        ("look_minus_x", math.pi, -106.1, "Right"),
    ]:
        await do(drone.rotate_to_yaw_async(yaw))
        time.sleep(0.3)
        score = green_pixels(read_frame(drone))
        print(f">> arrow {label}: green_pixels={score}")
        obs.append((score, branch_x, turn))
    score, branch_x, turn = max(obs, key=lambda x: x[0])
    if score < 200:
        raise RuntimeError(f"Could not confidently read green arrow: {obs}")
    return branch_x, turn

def sphere_count_for_branch(world, branch_x):
    obj = "SpherePuzzle_1" if branch_x > 0 else "SpherePuzzle_3"
    count = int(round(world.get_object_float_property(obj, "SphereCount")))
    if not (1 <= count <= 5):
        raise RuntimeError(f"Bad {obj}.SphereCount={count}")
    return count

def target_for_route(world, first, second):
    vehicle = {
        ("Left", "Left"): "tank",
        ("Left", "Right"): "boat",
        ("Right", "Left"): "jet",
        ("Right", "Right"): "icecream",
    }[(first, second)]

    # Coordinate map from latest build object layout.
    # First Left = +X branch; First Right = -X branch.
    # Second Left/Right are relative to travel direction after the first turn.
    if first == "Left":
        sx = 1
        sy = -1 if second == "Left" else 1
    else:
        sx = -1
        # Latest build room layout for the -X branch is mirrored relative to the
        # visible course heading: second Left lands in the north/+Y room, second
        # Right lands in the south/-Y room.
        sy = 1 if second == "Left" else -1

    objs = []
    for name in world.list_objects("GoalObject.*"):
        p = world.get_object_pose(name)["translation"]
        x, y, z = float(p["x"]), float(p["y"]), float(p["z"])
        if (x > 0) == (sx > 0) and (y > 0) == (sy > 0):
            objs.append((name, x, y, z, world.get_object_scale(name)))
    if len(objs) != 4:
        raise RuntimeError(f"Expected 4 goal objects in selected room, got {objs}")

    if vehicle == "jet":
        jets = [o for o in objs if max(o[4]) <= 1.1 and o[3] > -1]
        if len(jets) != 1:
            raise RuntimeError(f"Jet candidate ambiguity: {jets} from {objs}")
        chosen = jets[0]
    elif vehicle == "icecream":
        ground_scale1 = [o for o in objs if max(o[4]) <= 1.1]
        if first == "Right" and second == "Right":
            # In the latest build's right/right room, the ice-cream truck mesh is
            # the scale-1 object whose origin is reported at z ~= -1.89; the other
            # scale-1 object is the jet/decoy. Verified visually from FPV captures.
            ground_scale1 = [o for o in ground_scale1 if o[3] < -1]
        else:
            ground_scale1 = [o for o in ground_scale1 if o[3] > -1]
        if len(ground_scale1) != 1:
            raise RuntimeError(f"Icecream candidate ambiguity: {ground_scale1} from {objs}")
        chosen = ground_scale1[0]
    else:
        heavy = [o for o in objs if o[3] > -1 and min(o[4]) >= 1.9]
        if len(heavy) != 2:
            raise RuntimeError(f"Tank/boat candidate ambiguity: {heavy} from {objs}")
        # In +X rooms boat has smaller X and tank larger X; mirrored in -X rooms.
        if sx > 0:
            boat, tank = sorted(heavy, key=lambda o: o[1])
        else:
            tank, boat = sorted(heavy, key=lambda o: o[1])
        chosen = tank if vehicle == "tank" else boat
    return vehicle, chosen

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

async def solve(address="127.0.0.1"):
    client, world, drone = connect(address)
    try:
        drone.enable_api_control(); drone.arm()
        print(">> takeoff")
        await do(drone.takeoff_async())
        await do(drone.move_to_position_async(0, -35, -ALT, FAST))

        branch_x, first = await first_turn_from_camera(drone)
        count = sphere_count_for_branch(world, branch_x)
        second = "Left" if count % 2 == 0 else "Right"
        vehicle, target = target_for_route(world, first, second)
        name, tx, ty, tz, scale = target
        print(f">> decoded: first={first} branch_x={branch_x:.1f} spheres={count} second={second} vehicle={vehicle} target={name} xyz=({tx:.1f},{ty:.1f},{tz:.1f})")

        # Fly through the clue branch and into the selected final room.
        await do(drone.move_to_position_async(branch_x, -11.3, -ALT, FAST))
        await do(drone.move_to_position_async(branch_x, (108 if ty > 0 else -128), -ALT, FAST))
        await do(drone.move_to_position_async(tx, ty, -ALT, FAST))

        if vehicle == "icecream":
            print(">> landing beside ice-cream truck")
            # The README says the delivery trigger is landing on the ground within
            # 50 m of the ice-cream truck. Do not crash into it; stay at a safe
            # hover point beside the truck, then run the normal landing command.
            # Avoid extra takeoff/fallback attempts because they can leave a stale
            # ProjectAirSim task running after a timeout.
            offsets = [(0.0, 10.0 if ty < 0 else -10.0), (0.0, 0.0), (10.0, 0.0), (-10.0, 0.0), (0.0, -10.0 if ty < 0 else 10.0), (18.0, 18.0 if ty < 0 else -18.0), (-18.0, 18.0 if ty < 0 else -18.0)]
            result = "UNKNOWN"; elapsed = None
            for idx, (dx, dy) in enumerate(offsets, 1):
                lx, ly = tx + dx, ty + dy
                print(f">> landing point {idx}/{len(offsets)} beside {name}: ({lx:.1f},{ly:.1f})", flush=True)
                if idx > 1:
                    try:
                        drone.enable_api_control(); drone.arm()
                        await asyncio.wait_for(do(drone.takeoff_async()), timeout=10)
                    except Exception as e:
                        print(f">> retakeoff bounded: {type(e).__name__}: {e}", flush=True)
                await do(drone.move_to_position_async(lx, ly, -3.0, SLOW))
                try:
                    await asyncio.wait_for(do(drone.land_async()), timeout=35)
                except Exception as e:
                    print(f">> land command bounded: {type(e).__name__}: {e}", flush=True)
                    try:
                        drone.cancel_last_task()
                    except Exception:
                        pass
                result, elapsed = await poll_result(world, timeout=3)
                try:
                    kin = drone.get_estimated_kinematics()
                    pos = kin["pose"]["position"] if isinstance(kin, dict) else kin.pose.position
                    print(f">> landed_state={drone.get_landed_state()} est=({float(pos['x'] if isinstance(pos, dict) else pos.x):.1f},{float(pos['y'] if isinstance(pos, dict) else pos.y):.1f},{float(pos['z'] if isinstance(pos, dict) else pos.z):.1f}) result={result}", flush=True)
                except Exception as e:
                    print(f">> landed_state/pose read failed: {type(e).__name__}: {e}", flush=True)
                if result == "PASSED":
                    break
                if result == "FAILED":
                    break
            print(f">> RESULT {result} {elapsed}")
            if result != "PASSED":
                raise SystemExit(2)
            return
        else:
            print(f">> flying into {vehicle}")
            if vehicle == "jet":
                # Jets are airborne/large meshes. The object origin is close but not
                # always enough to trip the vehicle trigger, so sweep through the
                # jet body from both axes at several heights and poll after each
                # bounded impact attempt.
                async def jet_sweep(start, path, zlabel):
                    try:
                        await do(drone.move_to_position_async(start[0], start[1], start[2], FAST))
                        await asyncio.wait_for(do(drone.move_on_path_async(path, 8.0)), timeout=8)
                    except Exception as e:
                        print(f">> jet sweep {zlabel} ended/blocked: {type(e).__name__}: {e}")
                        try:
                            drone.cancel_last_task()
                        except Exception:
                            pass
                    r, et = await poll_result(world, timeout=1.0)
                    print(f">> jet sweep {zlabel} result={r} elapsed={et}", flush=True)
                    if r == "PASSED":
                        return True
                    if r == "FAILED":
                        raise SystemExit(2)
                    return False

                for hit_z in (-3.0, -2.2, -1.4, -0.6, 0.1):
                    # Sweep along Y at the jet's X coordinate. Do not sweep along X:
                    # the decoy ground vehicles in this room share nearly the same
                    # Y coordinate, and a low X-sweep can hit a trap first.
                    if await jet_sweep((tx, ty + 24.0, hit_z), [[tx, ty, hit_z], [tx, ty - 24.0, hit_z]], f"y@{hit_z}"):
                        return
            else:
                await do(drone.move_to_position_async(tx, ty, -1.2, SLOW))
                # Ground vehicles are low; finish at/just below object origin height so
                # the drone actually intersects the collision volume instead of flying
                # over it.
                try:
                    await asyncio.wait_for(do(drone.move_to_position_async(tx, ty, 0.4, SLOW)), timeout=8)
                except Exception as e:
                    print(f">> impact command ended/blocked as expected: {type(e).__name__}: {e}")
                    try:
                        drone.cancel_last_task()
                    except Exception:
                        pass

        result, elapsed = await poll_result(world)
        print(f">> RESULT {result} {elapsed}")
        if result != "PASSED":
            raise SystemExit(2)
    finally:
        client.disconnect()

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="EASY-mode RedRoad solver.")
    ap.add_argument("--address", default="127.0.0.1")
    ap.add_argument("--alt", type=float, default=ALT, help="flight altitude in metres")
    ap.add_argument("--speed", type=float, default=FAST, help="main move speed in m/s")
    ap.add_argument("--slow", type=float, default=SLOW, help="approach/landing speed in m/s")
    args = ap.parse_args()
    ALT = args.alt
    FAST = args.speed
    SLOW = args.slow
    asyncio.run(solve(args.address))
