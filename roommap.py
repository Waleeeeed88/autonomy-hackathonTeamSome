#!/usr/bin/env python3
"""Fly into the west room (north-spheres branch) and capture the 4 vehicles."""
import sys, asyncio, math, os, cv2
try: sys.stdout.reconfigure(encoding="utf-8")
except Exception: pass
from redteam_sim import connect, read_frame

os.makedirs("room", exist_ok=True)
async def do(c): await (await c)

async def main():
    client, world, drone = connect()
    try:
        drone.enable_api_control(); drone.arm()
        await do(drone.takeoff_async())
        await do(drone.move_to_position_async(0.0,-9.0,-5.0,5.0))   # onto the road near arrows
        await do(drone.move_to_position_async(95.0,-9.0,-5.0,8.0))  # north to spheres
        await do(drone.move_to_position_async(100.0,-50.0,-5.0,8.0))# west toward room
        for e in [-70,-90,-105,-120]:
            await do(drone.move_to_position_async(100.0,float(e),-5.0,5.0))
            await do(drone.hover_async()); await asyncio.sleep(0.4)
            gt=drone.get_ground_truth_kinematics()['pose']['position']
            tag=f"N{gt['x']:.0f}_E{gt['y']:.0f}"
            for yaw in [0,45,90,135,180,225,270,315]:
                await do(drone.rotate_to_yaw_async(math.radians(yaw)))
                await asyncio.sleep(0.3)
                f=read_frame(drone)
                if f is not None: cv2.imwrite(f"room/{tag}_y{yaw}.png",f)
            print(f"room scan at {tag}")
        print("roommap done")
    finally:
        client.disconnect()

asyncio.run(main())
