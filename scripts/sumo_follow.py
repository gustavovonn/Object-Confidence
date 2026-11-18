#!/usr/bin/env python3
"""Runs inside the generator's container (scripts/view_in_sumo.sh): sumo-gui on the LuST scenario as the
generator launches it, fast-forwarded to LEAD s before an onset, the camera tracking one vehicle.
Then the terminal steps the simulation, TraCI driving it, so sumo-gui's own play button stays idle.
The vehicle is magenta, red from the onset on; a red marker stays where it was at the onset.
Each line shows the speed v and the change since the previous step (0.1 s): angle (clockwise from north,
as SUMO) and speed, and a_lat = v * Δangle / 0.1 s, the quantity of error_diagnosis.py's limit.

Usage: sumo_follow.py <sumo id> <onset SUMO time [s]> <lead [s]> <seed> <SUMO --begin [s]> <sumocfg> [--gui false]
Run from the scenario directory; the SUMO configuration is the one of the generator's run (common.sumo_clock).
"""
import math
import sys

import traci

STEP = 0.1  # --step-length
VIEW = "View #0"
BEFORE = (255, 0, 255, 255)  # magenta
AFTER = (255, 0, 0, 255)     # red, from the onset on


def key(t):
    return round(t, 1)


def main():
    vid, onset, lead, seed = sys.argv[1], key(float(sys.argv[2])), float(sys.argv[3]), sys.argv[4]
    begin, sumocfg = sys.argv[5], sys.argv[6]
    gui = "--gui" not in sys.argv or sys.argv[sys.argv.index("--gui") + 1] != "false"
    traci.start(["sumo-gui" if gui else "sumo", "-c", sumocfg, "--seed", seed,
                 "--begin", begin, "--step-length", str(STEP), "--default.action-step-length", "1",
                 "--lanechange.duration", "3", "--no-warnings", "--no-step-log",
                 "--summary-output", "/tmp/summary.xml", "--tripinfo-output", "/tmp/tripinfo.xml",
                 "--log", "/tmp/sumo.log"] + (["--quit-on-end", "false"] if gui else []))

    prev = {}  # angle and speed of the vehicle one step back

    def step():
        if present():
            prev.update(angle=traci.vehicle.getAngle(vid), speed=traci.vehicle.getSpeed(vid))
        else:
            prev.clear()
        traci.simulationStep()

    def now():
        return key(traci.simulation.getTime())

    def present():
        return vid in traci.vehicle.getIDList()

    def decorate():
        if not present():
            return
        colour = AFTER if now() >= onset else BEFORE
        traci.vehicle.setColor(vid, colour)
        traci.vehicle.highlight(vid, colour, size=12)
        if now() >= onset and "onset" not in traci.poi.getIDList():
            print("*** ONSET ***")
            x, y = traci.vehicle.getPosition(vid)
            traci.poi.add("onset", x, y, AFTER, poiType="onset", layer=100, width=4, height=4)

    def status():
        if not present():
            return f"t={now():.1f}: {vid} not in the simulation"
        speed, angle = traci.vehicle.getSpeed(vid), traci.vehicle.getAngle(vid)
        line = f"t={now():.1f} (onset {now() - onset:+.1f} s)  v {speed:5.1f} m/s"
        if prev:
            d_angle = (angle - prev["angle"] + 180.0) % 360.0 - 180.0  # [deg]
            a_lat = speed * math.radians(abs(d_angle)) / STEP
            line += f"  Δangle {d_angle:+6.1f} deg  Δspeed {speed - prev['speed']:+5.2f} m/s  a_lat {a_lat:5.1f} m/s²"
        return line + f"  lane {traci.vehicle.getLaneID(vid)}"

    print(f"fast-forwarding to {onset - lead:.1f} ...")
    traci.simulationStep(onset - lead)
    # a vehicle that departs after onset - lead: on until it appears
    while not present() and now() < onset + 5:
        step()
    if not present():
        sys.exit(f"{vid} not in the simulation around {onset:.1f}")
    decorate()
    if gui:
        traci.gui.trackVehicle(VIEW, vid)
        traci.gui.setZoom(VIEW, 800)

    print("Enter: +0.1 s | <s>: s seconds | o: to the onset | q: quit")
    print(status())
    while True:
        try:
            cmd = input("> ").strip()
        except EOFError:
            break
        if cmd == "q":
            break
        if cmd == "":
            target = now() + STEP
        elif cmd == "o":
            target = onset
        else:
            try:
                target = now() + float(cmd)
            except ValueError:
                print("Enter, a number of seconds, o or q")
                continue
        # one step at a time, so the onset is never skipped
        while now() < key(target):
            step()
            decorate()
        print(status())
    traci.close()


if __name__ == "__main__":
    main()
