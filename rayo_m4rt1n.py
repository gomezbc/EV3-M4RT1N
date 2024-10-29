#!/usr/bin/env python3

# ABS(LEFT - RIGHT) >= 2000 GIRAR
# LEFT - RIGHT Y SI EL VALOR NEGATIVO => IZQ , POSITVO => DERECHA (O AL REVES)

from ev3dev2.sensor import Sensor, INPUT_3
from ev3dev2.motor import MoveTank, OUTPUT_C, OUTPUT_A

from time import sleep
SPEED = 40

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
    3: 1,
    4: 2,
    5: 3,
    6: 3,
    7: 4,
    8: 5,
    9: 10,
    10: 11,
}

indexPenalty ={
    0: 1,
    1: 1,
    2: 1,
    3: 1,
    4: 1,
    5: 1,
    6: 1,
    7: 3,
    8: 4,
    9: 7,
    10: 10,
}


## TENEMOS QUE MIRAR SIEMPRE EL CASO => 012 -- 765

def hasToSteer(left, right):
    return left >= 7000 or right >= 7000

def getNewSpeed(left, right):
    leftIndex = min(left // 1600, 10) 
    rightIndex = min(right // 1600, 10)
    leftSpeed = SPEED - motorSpeed[leftIndex]
    rightSpeed = SPEED - motorSpeed[rightIndex]
    if(leftSpeed > rightSpeed):
        rightSpeed = rightSpeed - indexPenalty[rightIndex]
    else:
        leftSpeed = leftSpeed - indexPenalty[leftIndex]
    return leftSpeed, rightSpeed


def getSensorWeigths(lsa):
    left = 0
    right = 0
    for i in range(0,4):
        left = left + (lsa.value(i) * sensorWeight[i])
    for i in range(4,8):
        right = right + (lsa.value(i) * sensorWeight[i])
    return left, right

def getMidSensorWeigths(lsa):
    mid = 0
    for i in range (3,5):
        mid = mid + (lsa.value(i)*sensorWeight[i]) # MAX 16000
    return mid

def main():
    tank_drive = MoveTank(OUTPUT_A, OUTPUT_C)
    tank_drive.on(SPEED,SPEED)
    lsa = Sensor(INPUT_3)
    while True:
        left, right = getSensorWeigths(lsa)
        mid = getMidSensorWeigths(lsa)
        if hasToSteer(left, right) and mid > 6500:
           speedLeft, speedRight = getNewSpeed(left, right)
           tank_drive.on(speedLeft, speedRight)
        elif left <= 6000 and right <= 6000:
            tank_drive.on(SPEED+10,SPEED+10)
        else:
            tank_drive.on(SPEED,SPEED)
        sleep(0.01)

if __name__ == "__main__":
    main()
