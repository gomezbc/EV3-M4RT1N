#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, MoveTank, OUTPUT_C, OUTPUT_A

from time import sleep

def forward(tank, v, tsec):
    tank.on_for_seconds(v, v, seconds = tsec)

def turnLeft(tank, vl, vr, tsec):
    # turn left on the spot
    tank.on_for_seconds(vl, vr, seconds = tsec)

def main():
    # left motor D
    # right motor A
    motor = MoveTank(OUTPUT_A, OUTPUT_C)
    for i in range(0, 4):
        forward(motor, 50, 3.11)
        turnLeft(motor, 37, -37, 0.5)
    print("Square finished")

# 149 - 130
# 31 3

# 51 54
if __name__=="__main__":
    main()
    
