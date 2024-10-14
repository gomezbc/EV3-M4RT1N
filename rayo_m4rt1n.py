#!/usr/bin/env python3

# ABS(LEFT - RIGHT) >= 2000 GIRAR
# LEFT - RIGHT Y SI EL VALOR NEGATIVO => IZQ , POSITVO => DERECHA (O AL REVES)

from ev3dev2.sensor import Sensor, INPUT_3
from ev3dev2.motor import MoveTank, OUTPUT_C, OUTPUT_A

from time import sleep
SPEED = 15

sensorWeight = {
    0: 30,
    1: 40,
    2: 60,
    3: 80,
    4: 80,
    5: 60,
    6: 40,
    7: 30
}

motorSpeed = {
    0: 0,
    1: 0,
    2: 0,
    3: 0,
    4: 1,
    5: 2,
    6: 3,
    7: 4,
    8: 5,
    9: 6,
    10: 6,
}


def hasToSteer(left, right):
    return abs(left - right) > 3000

def getNewSpeed(left, right):
    leftMod = (left // 1000) % 10
    speedLeft = (left // 1000) - leftMod
    rightMod = (right // 1000) % 10
    speedRight = (right // 1000) - rightMod
    return SPEED - motorSpeed[speedLeft], SPEED - motorSpeed[speedRight]


def getSensorWeigths(lsa):
    left = 0
    right = 0
    for i in range(0,4):
        left = left + (lsa.value(i) * sensorWeight[i])
    for i in range(4,8):
        right = right + (lsa.value(i) * sensorWeight[i])
    return left, right

def main():
    tank_drive = MoveTank(OUTPUT_A, OUTPUT_C)
    tank_drive.on(SPEED,SPEED)
    lsa = Sensor(INPUT_3)
    i=0
    while True:
        left, right = getSensorWeigths(lsa)
        print("  L " + str(left), "   " + str(right))
        if hasToSteer(left, right):
           speedLeft, speedRight = getNewSpeed(left, right)
           tank_drive.on(speedLeft, speedRight)
        else:
            tank_drive.on(SPEED,SPEED)
        i += 1
        sleep(0.05)

if __name__ == "__main__":
    main()