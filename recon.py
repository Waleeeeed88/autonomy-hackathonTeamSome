#!/usr/bin/env python3
"""Focused recon: scan the start area at low altitude, all yaws, to locate arrows."""
import sys, asyncio, math, time, os, cv2
try: sys.stdout.reconfigure(encoding="utf-8")
except Exception: pass
from redteam_sim import connect, read_frame

os.makedirs("recon", exist_ok=True)
_i=[0]
def save(f,tag):
    if f is None: return
    cv2.imwrite(f"recon/{_i[0]:03d}_{tag}.png", f); _i[0]+=1

async def do(c): await (await c)

async def main():
    client, world, drone = connect()
    try:
        drone.enable_api_control(); drone.arm()
        await do(drone.takeoff_async())
        # Low altitude recon at several north positions, full 360 yaw scan
        for n in [0, 10, 20, 30, 40, 50]:
            await do(drone.move_to_position_async(float(n), -35.0, -2.5, 3.0))
            await do(drone.hover_async()); await asyncio.sleep(0.5)
            for yaw in [0, 45, 90, 135, 180, -135, -90, -45]:
                await do(drone.rotate_to_yaw_async(math.radians(yaw)))
                await asyncio.sleep(0.4)
                f = read_frame(drone)
                save(f, f"N{n}_y{yaw}")
            print(f"scanned N={n}")
        print("recon done")
    finally:
        client.disconnect()

asyncio.run(main())
