#!/usr/bin/env python3

# ABS(LEFT - RIGHT) >= 2000 GIRAR
# LEFT - RIGHT Y SI EL VALOR NEGATIVO => IZQ , POSITVO => DERECHA (O AL REVES)

from ev3dev2.sensor import Sensor, INPUT_3
from ev3dev2.motor import MoveTank, OUTPUT_C, OUTPUT_A

from time import sleep
SPEED = 55

sensorWeight = {
    0: 7,
    1: 5,
    2: 3,
    3: 1,
    4: 1,
    5: 3,
    6: 5,
    7: 7
}

motorSpeed = {
    0: 2,
    1: 4,
    2: 4,
    3: 4,
    4: 5,
    5: 6,
    6: 8,
    7: 10,
    8: 12,
    9: 16,
    10: 20,
}

LOGS_FILE = 'logs.txt'

def writeInfo(left, right):
    with open(LOGS_FILE, 'a') as f:
        f.write("L: "+str(left) + " R: " + str(right) + " ABS: " + str(abs(left-right))+"\n")

def hasToSteer(left, right):
    writeInfo(left, right)
    return abs(left - right) > 200

def getNewSpeed(left, right):
    # mirar como ajustar mejor la velocidad de la rueda que no tiene que corregir
    speedLeft = (left // 160)
    speedRight = (right // 160)
    if(speedRight > speedLeft):
         speedLeft = speedLeft//2
    else:
        speedRight = speedRight//2
    with open(LOGS_FILE, 'a') as f:
        f.write("speedLeft: "+str(speedLeft) + " speedRight: " + str(speedRight) + "\n")
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
    while True:
        left, right = getSensorWeigths(lsa)
        if hasToSteer(left, right):
           speedLeft, speedRight = getNewSpeed(left, right)
           tank_drive.on(speedLeft, speedRight)
        else:
            tank_drive.on(SPEED,SPEED)
        sleep(0.05)

if __name__ == "__main__":
    main()
