#!/usr/bin/env python3

from ev3dev2.motor import MoveDifferential, SpeedRPM, OUTPUT_C, OUTPUT_A
from ev3dev2.wheel import EV3EducationSetTire # gurpilaren zirkunferentzia: 176mm

from time import sleep

def main():
    mdiff = MoveDifferential(OUTPUT_A, OUTPUT_C, EV3EducationSetTire, 16)
    mdiff.odometry_start(theta_degrees_start=0.0)
    for i in range(0, 4):
        mdiff.on_to_coordinates(SpeedRPM(40), 0, 600)
        mdiff.turn_to_angle(SpeedRPM(40), 90)
    mdiff.on_to_coordinates(SpeedRPM(40), 0, 0)
    mdiff.turn_to_angle(SpeedRPM(40), 90)


if __name__=="__main__":
    main()
