#!/usr/bin/env python3
"""
Waypoint calibration helper.
Run AFTER  python solution.py --explore  has created debug_frames/*.

Reads every frame, applies the same CV as solution.py, and reports:
  - Which frames contain a detectable green arrow (and direction)
  - Which frames contain blue spheres (and count)
  - Frame filenames so you can map them to checkpoint waypoints

Then copy the detected positions back into solution.py WAYPOINTS.

Usage:
    python tune_waypoints.py
    python tune_waypoints.py --frames debug_frames
"""

import sys
import argparse
import os
from collections import Counter
import cv2
import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


def _morph(m, ko=3, kc=9):
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  np.ones((ko, ko), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((kc, kc), np.uint8))
    return m

def seg_green(hsv):
    return _morph(cv2.inRange(hsv, np.array([35,70,70]), np.array([85,255,255])))

def seg_red(hsv):
    r1 = cv2.inRange(hsv, np.array([  0,100,80]), np.array([ 12,255,255]))
    r2 = cv2.inRange(hsv, np.array([158,100,80]), np.array([180,255,255]))
    return _morph(cv2.bitwise_or(r1, r2))

def seg_blue(hsv):
    return _morph(cv2.inRange(hsv, np.array([90,80,60]),
                                    np.array([135,255,255])), ko=5, kc=7)

def biggest(mask, min_area=300):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in cnts if cv2.contourArea(c) >= min_area]
    return max(valid, key=cv2.contourArea) if valid else None

def cx(cnt):
    M = cv2.moments(cnt)
    return int(M["m10"] / M["m00"]) if M["m00"] else None

def detect_arrow(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gc  = biggest(seg_green(hsv))
    rc  = biggest(seg_red(hsv))
    if gc is None:
        return None
    gcx = cx(gc)
    if gcx is None:
        return None
    if rc is not None:
        rcx = cx(rc)
        if rcx is not None:
            return 'left' if gcx < rcx else 'right'
    # fallback
    fw = frame.shape[1]
    if gcx < fw // 2 - fw // 8:
        return 'left'
    if gcx > fw // 2 + fw // 8:
        return 'right'
    return None

def count_spheres(frame):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    bm   = seg_blue(hsv)
    cnts, _ = cv2.findContours(bm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return len([c for c in cnts if cv2.contourArea(c) > 150])


def analyse(frame_dir):
    files = sorted(f for f in os.listdir(frame_dir)
                   if f.lower().endswith(('.png', '.jpg')))
    if not files:
        print(f"No frames found in {frame_dir}/")
        return

    print(f"Analysing {len(files)} frames in {frame_dir}/\n")
    print(f"{'File':<50}  {'Arrow':>8}  {'Spheres':>8}")
    print("-" * 72)

    arrow_detections  = Counter()
    sphere_detections = Counter()

    for fname in files:
        path  = os.path.join(frame_dir, fname)
        frame = cv2.imread(path)
        if frame is None:
            continue
        arrow   = detect_arrow(frame)
        spheres = count_spheres(frame)

        arrow_str  = arrow if arrow else "-"
        sphere_str = str(spheres) if spheres > 0 else "-"

        if arrow or spheres > 0:
            print(f"{fname:<50}  {arrow_str:>8}  {sphere_str:>8}")

        if arrow:
            arrow_detections[arrow] += 1
        if spheres > 0:
            sphere_detections[spheres] += 1

    print()
    print(f"Arrow detections:  {dict(arrow_detections)}")
    print(f"Sphere detections: {dict(sphere_detections)}")
    if arrow_detections:
        best = arrow_detections.most_common(1)[0][0]
        print(f"\n→ Majority arrow direction: {best.upper()}")
    if sphere_detections:
        best = sphere_detections.most_common(1)[0][0]
        parity = 'even' if best % 2 == 0 else 'odd'
        turn   = 'LEFT' if best % 2 == 0 else 'RIGHT'
        print(f"→ Most common sphere count: {best} ({parity}) → second turn: {turn}")

    print("\nUse the checkpoint tags in the filenames (explore_cp0, explore_cp1 …)")
    print("to identify which waypoint each frame came from, then update WAYPOINTS")
    print("in solution.py accordingly.\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", default="debug_frames",
                    help="directory of frames from --explore run")
    args = ap.parse_args()
    analyse(args.frames)


if __name__ == "__main__":
    main()
