#!/usr/bin/env python3
"""High-altitude 360 survey to map the course layout."""
import sys, asyncio, math, os, cv2
try: sys.stdout.reconfigure(encoding="utf-8")
except Exception: pass
from redteam_sim import connect, read_frame

os.makedirs("survey", exist_ok=True)
async def do(c): await (await c)

async def main():
    client, world, drone = connect()
    try:
        drone.enable_api_control(); drone.arm()
        await do(drone.takeoff_async())
        # climb high above start
        await do(drone.move_to_position_async(0.0,-35.0,-45.0,5.0))
        await do(drone.hover_async()); await asyncio.sleep(1.0)
        for yaw in range(0,360,15):
            await do(drone.rotate_to_yaw_async(math.radians(yaw)))
            await asyncio.sleep(0.3)
            f=read_frame(drone)
            if f is not None: cv2.imwrite(f"survey/y{yaw:03d}.png",f)
        print("survey done")
    finally:
        client.disconnect()

asyncio.run(main())
