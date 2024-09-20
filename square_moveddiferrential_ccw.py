#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, MoveTank, OUTPUT_C, OUTPUT_A

from time import sleep

def main():
    motor = MoveTank(OUTPUT_A, OUTPUT_C)
    while True:
        motor.on_for_rotations(left_speed=50, right_speed = 50, rotations = 4.5)
        motor.on_for_rotations(left_speed=100, right_speed = 0, rotations = 1.15)


if __name__=="__main__":
    main()
    
