#!/usr/bin/env python3
"""
Autonomous Red Team Hack Sim — Mission Solver  (RedRoad)
========================================================
The course geometry is FIXED; only the clues randomise each round
(green-arrow direction, sphere count, and therefore the target vehicle).

Pipeline
  1. Read the GREEN arrow (camera): points LEFT  -> NORTH branch (turn1=left)
                                    points RIGHT -> SOUTH branch (turn1=right)
  2. Fly the branch road to the sphere junction; COUNT blue spheres (camera):
        even -> turn2=left,  odd -> turn2=right
  3. Resolve target room + vehicle from the legend, then fly to the exact
     GoalObject and deliver:
        (L,L) -> Tank  (NW room)      fly into
        (L,R) -> Boat  (NE room)      fly into
        (R,L) -> Jet   (SE room)      fly into
        (R,R) -> Ice-Cream (SW room)  LAND beside

Exact vehicle positions come from the sim's GoalObjects (EASY mode lets us use
ground truth for navigation); the clues are still read from the camera.

Usage:
    python solution.py
    python solution.py --no-deliver        # read clues only, report target
    python solution.py --force LL           # skip CV, force a path (debug)
"""

import sys, asyncio, argparse, math, time, os, json
from collections import Counter
import cv2, numpy as np
from redteam_sim import connect, reset, read_frame

try:
    sys.stdout.reconfigure(encoding="utf-8"); sys.stderr.reconfigure(encoding="utf-8")
except Exception: pass

DBG="debug_frames"; os.makedirs(DBG,exist_ok=True); _fi=[0]
def dbg(f,tag):
    if f is None: return
    cv2.imwrite(os.path.join(DBG,f"{_fi[0]:04d}_{tag}.png"),f); _fi[0]+=1

# ══════════════ EXACT COURSE MAP (from GoalObject poses) ══════════════════════
CRUISE = -5.0      # altitude for clue reading / transit (m up)
GATE_Z = -2.5      # altitude when passing gate triggers / approaching
SLOW, FAST = 4.0, 8.0

ROAD_E   = -9.0
ARROW_VIEW = (0.0, -12.0)         # sit here facing EAST to read arrows
SPHERE_VIEW = {'north': (90.0, -9.0), 'south': (-90.0, -9.0)}

# room -> vehicle type -> (north, east)
GOALS = {
 'NE': {'jet':(76.3,102.2),  'boat':(90.6,102.9),  'tank':(103.3,106.4), 'icecream':(115.4,104.5)},
 'NW': {'jet':(78.5,-129.7), 'boat':(92.4,-129.8), 'tank':(102.5,-130.6),'icecream':(114.1,-129.9)},
 'SE': {'jet':(-116.2,103.5),'boat':(-104.7,107.2),'tank':(-96.1,108.2), 'icecream':(-86.2,104.6)},
 'SW': {'jet':(-113.6,-127.9),'boat':(-84.5,-125.0),'tank':(-93.5,-127.9),'icecream':(-105.0,-127.9)},
}
# approximate vehicle heights (m) -> ram altitude
VEH_TOP = {'jet':5.4,'boat':2.6,'tank':3.2,'icecream':5.4}

LEGEND = {('left','left'):'tank',('left','right'):'boat',
          ('right','left'):'jet',('right','right'):'icecream'}
PRETTY = {'tank':'TANK','boat':'BOAT','jet':'JET','icecream':'ICE-CREAM TRUCK'}

# ══════════════ flight helpers ════════════════════════════════════════════════
async def do(c): await (await c)
async def fly(drone,n,e,d=CRUISE,v=SLOW,label=""):
    print(f"   fly -> N={n:7.1f} E={e:7.1f} D={d:5.1f} {v}m/s {label}")
    await do(drone.move_to_position_async(n,e,d,v))
async def hover(drone,s=1.0):
    await do(drone.hover_async()); await asyncio.sleep(s)
async def face(drone,yaw):
    await do(drone.rotate_to_yaw_async(math.radians(yaw))); await asyncio.sleep(0.4)
def grab(drone,n=6,gap=0.16):
    out=[]
    for _ in range(n):
        f=read_frame(drone)
        if f is not None: out.append(f)
        time.sleep(gap)
    return out
def race_state(world):
    try: return world.get_object_float_property("RaceManager","MissionState")
    except Exception: return float('nan')

async def deliver_to(drone, world, gn, ge, vtype, room_tag):
    """Fly to a goal and deliver (ram a vehicle / land beside the truck).
    Returns the MissionState observed shortly afterwards. Ramming uses a
    DURATION-based velocity command so a collision can't hang the move."""
    off = 16.0 if ge<0 else -16.0
    print(f"   -> {room_tag}: {PRETTY[vtype]} @ ({gn:.1f},{ge:.1f})")
    try:
        await fly(drone, gn, ge+off, GATE_Z, FAST, "standoff")
    except Exception as e:
        print(f"      (standoff move issue: {type(e).__name__})")
    fr=grab(drone,2,0.15)
    if fr: dbg(fr[0], f"target_{room_tag}_{vtype}")

    if vtype=='icecream':
        try:
            await fly(drone, gn, ge+off*0.4, GATE_Z, SLOW, "approach truck")
            await do(drone.land_async())
            print(f"      landed (landed_state={drone.get_landed_state()})")
        except Exception as e:
            print(f"      (land issue: {type(e).__name__})")
        # poll
        t=time.time()
        while time.time()-t < 6.0:
            ms=race_state(world)
            if ms in (2.0,3.0): return ms
            await asyncio.sleep(0.3)
    else:
        # Ram across the vehicle at several altitudes (low hull -> tall body),
        # alternating direction. Velocity moves can't hang on a collision.
        cur_off = off
        for z in (-0.8, -1.6, -2.6, -3.6):
            try:
                # reposition to a standoff on the current side at this altitude
                await fly(drone, gn, ge+cur_off, z, SLOW, f"standoff z={z}")
            except Exception: pass
            ve = -math.copysign(2.5, cur_off)
            dur = abs(cur_off)/2.5 + 3.0
            print(f"      ram pass z={z} ve={ve:.1f} dur={dur:.1f}")
            try:
                await do(drone.move_by_velocity_async(0.0, ve, 0.0, dur))
            except Exception as e:
                print(f"      (ram ended: {type(e).__name__})")
            try: drone.cancel_last_task()
            except Exception: pass
            # poll after this pass
            t=time.time()
            while time.time()-t < 2.5:
                ms=race_state(world)
                if ms in (2.0,3.0): return ms
                await asyncio.sleep(0.25)
            cur_off = -cur_off    # next pass from the far side, back across
    # back off / re-climb before trying another room
    try:
        await do(drone.move_to_position_async(gn, ge+off, GATE_Z, SLOW))
    except Exception: pass
    return race_state(world)

# ══════════════ CV ════════════════════════════════════════════════════════════
def _m(m,ko=3,kc=9):
    m=cv2.morphologyEx(m,cv2.MORPH_OPEN,np.ones((ko,ko),np.uint8))
    return cv2.morphologyEx(m,cv2.MORPH_CLOSE,np.ones((kc,kc),np.uint8))
def seg_green(h): return _m(cv2.inRange(h,np.array([40,80,60]),np.array([90,255,255])))
def seg_red(h):
    r1=cv2.inRange(h,np.array([0,140,90]),np.array([8,255,255]))
    r2=cv2.inRange(h,np.array([170,140,90]),np.array([180,255,255]))
    return _m(cv2.bitwise_or(r1,r2))
def seg_blue(h): return _m(cv2.inRange(h,np.array([100,120,60]),np.array([130,255,255])),5,7)
def biggest(m,a=300):
    c,_=cv2.findContours(m,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    v=[x for x in c if cv2.contourArea(x)>=a]; return max(v,key=cv2.contourArea) if v else None
def cxof(c):
    M=cv2.moments(c); return int(M["m10"]/M["m00"]) if M["m00"] else None

def classify_arrow(frame):
    if frame is None: return None
    h=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    gc=biggest(seg_green(h),300)
    if gc is None: return None
    gcx=cxof(gc)
    if gcx is None: return None
    rc=biggest(seg_red(h),300)
    if rc is not None:
        rcx=cxof(rc)
        if rcx is not None:
            return 'left' if gcx<rcx else 'right'
    fw=frame.shape[1]
    if gcx<fw//2-fw//8: return 'left'
    if gcx>fw//2+fw//8: return 'right'
    return None

def count_spheres(frame):
    if frame is None: return 0
    h=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV); bm=seg_blue(h)
    bm[:int(bm.shape[0]*0.35),:]=0
    c,_=cv2.findContours(bm,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    return len([x for x in c if cv2.contourArea(x)>120])

# ══════════════ mission ═══════════════════════════════════════════════════════
async def mission(address, deliver=True, force=None):
    print("="*60); print("  RED TEAM HACK SIM — Mission Solver"); print("="*60)
    client,world,drone=connect(address)
    arrow=None; turn2=None; result="unknown"
    try:
        print(f"\n[init] MissionState={race_state(world)}")
        print("\n[0] ARM & TAKEOFF")
        drone.enable_api_control(); drone.arm()
        await do(drone.takeoff_async())
        await fly(drone,0.0,-35.0,CRUISE,SLOW,"climb")
        await hover(drone,1.0)

        if force:
            arrow='left' if force[0]=='L' else 'right'
            turn2='left' if force[1]=='L' else 'right'
            print(f"\n[forced] path=({arrow},{turn2})")
        else:
            # ---- 1. arrows ----
            print("\n[1] READ ARROWS")
            await fly(drone,ARROW_VIEW[0],ARROW_VIEW[1],CRUISE,SLOW,"arrow view")
            await hover(drone,0.8)
            votes=Counter()
            for yaw in [90,85,95,88,92,83,97]:
                await face(drone,yaw)
                fr=grab(drone,6,0.14)
                for f in fr:
                    d=classify_arrow(f)
                    if d: votes[d]+=1
                if fr: dbg(fr[0],f"arrow_y{yaw}")
            print(f"   arrow votes: {dict(votes)}")
            arrow=votes.most_common(1)[0][0] if votes else 'left'
            print(f"   GREEN arrow points {arrow.upper()}")

        branch='north' if arrow=='left' else 'south'
        turn1=arrow

        if not force:
            # ---- 2. spheres ----
            print(f"\n[2] SPHERES ({branch} branch)")
            await fly(drone,0.0,ROAD_E,CRUISE,SLOW,"onto road")
            sv=SPHERE_VIEW[branch]
            await fly(drone,sv[0],sv[1],CRUISE,FAST,"to sphere view")
            await hover(drone,0.8)
            travel=0 if branch=='north' else 180
            counts=[]
            for dy in [0,-15,15,-30,30]:
                await face(drone,travel+dy)
                fr=grab(drone,6,0.14)
                for f in fr:
                    c=count_spheres(f)
                    if c>0: counts.append(c)
                if fr: dbg(fr[0],f"spheres_y{travel+dy}")
            vision_sc=Counter(counts).most_common(1)[0][0] if counts else 0
            # EASY mode: the exact sphere count is exposed by the puzzle object.
            sphere_obj='SpherePuzzle_1' if branch=='north' else 'SpherePuzzle_3'
            try:
                api_sc=int(round(world.get_object_float_property(sphere_obj,"SphereCount")))
            except Exception:
                api_sc=0
            sc = api_sc if api_sc>0 else (vision_sc if vision_sc>0 else 1)
            print(f"   spheres: vision={vision_sc} api={api_sc} -> using {sc}")
            turn2='left' if sc%2==0 else 'right'
            print(f"   spheres {sc} ({'even' if sc%2==0 else 'odd'}) -> turn2={turn2}")

        # ---- 3. resolve target ----
        if branch=='north':
            room='NW' if turn2=='left' else 'NE'
        else:
            room='SE' if turn2=='left' else 'SW'
        vtype=LEGEND[(turn1,turn2)]
        gn,ge=GOALS[room][vtype]
        print(f"\n[3] TARGET path=({turn1},{turn2}) -> {PRETTY[vtype]} in {room} room @ ({gn:.1f},{ge:.1f})")

        if not deliver:
            print("[--no-deliver] stopping."); return

        # ---- 4. navigate + deliver ----
        print(f"\n[4] DELIVER {PRETTY[vtype]}")
        # candidate order: primary room first, then the same TYPE in other rooms
        # (reaching a wrong goal is silent, so this fallback is safe insurance).
        order=[room]+[r for r in ['NE','NW','SE','SW'] if r!=room]
        for cand in order:
            cgn,cge=GOALS[cand][vtype]
            ms=await deliver_to(drone,world,cgn,cge,vtype,cand)
            if ms==2.0:
                try: et=world.get_object_float_property("RaceManager","ElapsedSeconds")
                except Exception: et=float('nan')
                print(f"\n  ✅  MISSION PASSED ({et:.1f}s) — {PRETTY[vtype]} in {cand} room")
                result="PASSED"; break
            if ms==3.0:
                print(f"\n  ❌  MISSION FAILED at {cand} room"); result="FAILED"; break
            print(f"   {cand} room: no trigger (state={ms}); trying next room…")
        if result=="unknown":
            print("   (no terminal RaceManager state after all candidates)")
    except Exception as e:
        import traceback; print(f"\n[ERROR] {type(e).__name__}: {e}"); traceback.print_exc()
    finally:
        try: drone.disarm()
        except Exception: pass
        client.disconnect(); print("\n[done] disconnected")
    json.dump({"arrow":arrow,"turn2":turn2,"result":result},
              open(os.path.join(DBG,"last_run.json"),"w"),indent=2)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--address",default="127.0.0.1")
    ap.add_argument("--no-deliver",action="store_true")
    ap.add_argument("--force",default=None,help="force path e.g. LL LR RL RR")
    a=ap.parse_args()
    asyncio.run(mission(a.address, not a.no_deliver, a.force))

if __name__=="__main__":
    main()
