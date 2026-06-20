#!/usr/bin/env python3
"""Diagnostic: read clues, deliver precisely to computed goal, log everything.
If the primary goal doesn't trigger PASS within a few seconds, try the same
vehicle TYPE in the other rooms (to discover the correct room mapping).
Wrong-goal behaviour (silent vs FAIL) is revealed by the logs."""
import sys, asyncio, math, time, os
from collections import Counter
import cv2, numpy as np
try: sys.stdout.reconfigure(encoding="utf-8")
except Exception: pass
from redteam_sim import connect, read_frame
import solution as S

async def do(c): await (await c)
def rs(world):
    try: return world.get_object_float_property("RaceManager","MissionState")
    except Exception: return float('nan')

GOALS=S.GOALS; LEGEND=S.LEGEND; PRETTY=S.PRETTY; VEH_TOP=S.VEH_TOP

async def deliver(drone, world, gn, ge, vtype, tag):
    off = 16.0 if ge<0 else -16.0
    print(f"   -> {tag}: goal ({gn:.1f},{ge:.1f}) type={vtype}")
    await do(drone.move_to_position_async(gn, ge+off, -2.5, 6.0))
    if vtype=='icecream':
        await do(drone.move_to_position_async(gn, ge+off*0.4, -2.5, 3.0))
        await do(drone.land_async())
        print(f"      landed; landed_state={drone.get_landed_state()}")
    else:
        z=-(VEH_TOP[vtype]*0.5)
        await do(drone.move_to_position_async(gn, ge+off, z, 3.0))
        await do(drone.move_to_position_async(gn, ge, z, 2.0))
        await do(drone.move_to_position_async(gn, ge-off*0.3, z, 2.0))
    # poll
    t=time.time(); last=None
    while time.time()-t<6:
        ms=rs(world)
        if ms!=last: print(f"      MissionState={ms}"); last=ms
        if ms in (2.0,3.0): return ms
        await asyncio.sleep(0.3)
    return rs(world)

async def main():
    client,world,drone=connect()
    try:
        print("init MissionState",rs(world))
        # also read API sphere counts
        for sp in ['SpherePuzzle_1','SpherePuzzle_3']:
            try: print(sp,"SphereCount",world.get_object_float_property(sp,"SphereCount"))
            except Exception as e: print(sp,"err",e)
        drone.enable_api_control(); drone.arm()
        await do(drone.takeoff_async())
        await do(drone.move_to_position_async(0,-35,-5,4))
        # read arrow
        votes=Counter()
        await do(drone.move_to_position_async(0,-12,-5,4))
        for yaw in [90,85,95,88,92]:
            await do(drone.rotate_to_yaw_async(math.radians(yaw))); await asyncio.sleep(0.3)
            for f in (read_frame(drone) for _ in range(5)):
                d=S.classify_arrow(f)
                if d: votes[d]+=1
            time.sleep(0.1)
        arrow=votes.most_common(1)[0][0] if votes else 'left'
        print("ARROW",dict(votes),"->",arrow)
        branch='north' if arrow=='left' else 'south'
        # read spheres (vision) + API
        await do(drone.move_to_position_async(0,-9,-5,4))
        sv=S.SPHERE_VIEW[branch]
        await do(drone.move_to_position_async(sv[0],sv[1],-5,8))
        travel=0 if branch=='north' else 180
        counts=[]
        for dy in [0,-15,15,-30,30]:
            await do(drone.rotate_to_yaw_async(math.radians(travel+dy))); await asyncio.sleep(0.3)
            for _ in range(5):
                c=S.count_spheres(read_frame(drone))
                if c>0: counts.append(c)
                time.sleep(0.05)
        sc=Counter(counts).most_common(1)[0][0] if counts else 1
        api_sc=world.get_object_float_property('SpherePuzzle_1' if branch=='north' else 'SpherePuzzle_3','SphereCount')
        print(f"SPHERES vision={sc} api={api_sc} samples={sorted(counts)}")
        turn2='left' if sc%2==0 else 'right'
        turn1=arrow
        vtype=LEGEND[(turn1,turn2)]
        print(f"PATH=({turn1},{turn2}) TYPE={PRETTY[vtype]}")
        print(f"MissionState before delivery: {rs(world)}")

        # primary room guess
        if branch=='north': primary='NW' if turn2=='left' else 'NE'
        else: primary='SE' if turn2=='left' else 'SW'
        order=[primary]+[r for r in ['NE','NW','SE','SW'] if r!=primary]
        for room in order:
            gn,ge=GOALS[room][vtype]
            ms=await deliver(drone,world,gn,ge,vtype,f"room {room}")
            print(f"   after {room}: MissionState={ms}")
            if ms==2.0:
                print(f"\n  ✅ PASSED at room {room} ({PRETTY[vtype]})"); break
            if ms==3.0:
                print(f"\n  ❌ FAILED at room {room}"); break
    finally:
        try: drone.disarm()
        except Exception: pass
        client.disconnect()

asyncio.run(main())
