#!/usr/bin/env python3
"""Grid-map the course: fly a grid, save frames tagged with ground-truth pos."""
import sys, asyncio, math, os, cv2
try: sys.stdout.reconfigure(encoding="utf-8")
except Exception: pass
from redteam_sim import connect, read_frame

os.makedirs("grid", exist_ok=True)
async def do(c): await (await c)

async def main():
    client, world, drone = connect()
    try:
        drone.enable_api_control(); drone.arm()
        await do(drone.takeoff_async())
        await do(drone.move_to_position_async(0.0,-35.0,-5.0,5.0))
        # grid over the far region where vehicles/rooms are
        norths = [40, 70, 100, 130]
        easts  = [-70, -40, -10, 20, 50]
        for n in norths:
            row = easts if (norths.index(n)%2==0) else list(reversed(easts))
            for e in row:
                await do(drone.move_to_position_async(float(n),float(e),-5.0,7.0))
                await do(drone.hover_async()); await asyncio.sleep(0.3)
                gt=drone.get_ground_truth_kinematics()['pose']['position']
                tag=f"N{gt['x']:.0f}_E{gt['y']:.0f}"
                for yaw in [0,90,180,270]:
                    await do(drone.rotate_to_yaw_async(math.radians(yaw)))
                    await asyncio.sleep(0.3)
                    f=read_frame(drone)
                    if f is not None: cv2.imwrite(f"grid/{tag}_y{yaw}.png",f)
                print(f"mapped {tag}")
        print("gridmap done")
    finally:
        client.disconnect()

asyncio.run(main())
