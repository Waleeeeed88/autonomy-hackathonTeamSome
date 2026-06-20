#!/usr/bin/env python3
"""Recon 2: approach the red/green arrows (east of start) for a clean read."""
import sys, asyncio, math, os, cv2
try: sys.stdout.reconfigure(encoding="utf-8")
except Exception: pass
from redteam_sim import connect, read_frame, reset

os.makedirs("recon2", exist_ok=True)
_i=[0]
def save(f,tag):
    if f is None: return
    cv2.imwrite(f"recon2/{_i[0]:03d}_{tag}.png", f); _i[0]+=1

async def do(c): await (await c)

async def main():
    client, world, drone = connect()
    try:
        drone.enable_api_control(); drone.arm()
        await do(drone.takeoff_async())
        await do(drone.move_to_position_async(0.0,-35.0,-4.0,3.0))
        # Sweep east, looking around at each stop
        for (n,e) in [(0,-35),(0,-10),(0,15),(0,40),(20,40),(40,40),(20,15)]:
            await do(drone.move_to_position_async(float(n),float(e),-4.0,4.0))
            await do(drone.hover_async()); await asyncio.sleep(0.4)
            for yaw in [0,30,60,90,120,-30,-60,-90,180]:
                await do(drone.rotate_to_yaw_async(math.radians(yaw)))
                await asyncio.sleep(0.35)
                save(read_frame(drone), f"N{n}_E{e}_y{yaw}")
            gt=drone.get_ground_truth_kinematics()['pose']['position']
            print(f"at N={gt['x']:.1f} E={gt['y']:.1f}")
        print("recon2 done")
    finally:
        client.disconnect()

asyncio.run(main())
