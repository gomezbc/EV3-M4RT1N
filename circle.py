#!/usr/bin/env python3

from ev3dev2.motor import MoveDifferential, SpeedRPM, OUTPUT_C, OUTPUT_A
from ev3dev2.wheel import EV3EducationSetTire # wheel gurpilaren zirkunferentzia: 176mm

from time import sleep

side_length = 800  # mm (80 cm)

def main():
    mdiff = MoveDifferential(OUTPUT_A, OUTPUT_C, EV3EducationSetTire, 118)
    mdiff.odometry_start(theta_degrees_start=0.0, x_pos_start=0.0, y_pos_start=0.0, sleep_time=0.005)
    mdiff.on_arc_right(SpeedRPM(80), 400, 2*1256)
    mdiff.odometry_stop()

if __name__ == "__main__":
    main()