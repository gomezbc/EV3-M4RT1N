#!/usr/bin/env python3

from ev3dev2.sensor import Sensor, INPUT_3
from ev3dev2.motor import MoveTank, OUTPUT_C, OUTPUT_A
from time import sleep

SPEED = 40

SENSOR_WEIGHT = {0: 25, 1: 35, 2: 55, 3: 80, 4: 80, 5: 65, 6: 45, 7: 35}
MOTOR_SPEED = {0: 0, 1: 0, 2: 0, 3: 1, 4: 2, 5: 3, 6: 3, 7: 4, 8: 5, 9: 10, 10: 11}
INDEX_PENALTY = {0: 1, 1: 1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 1, 7: 3, 8: 4, 9: 7, 10: 10}

def hasToSteer(left, right):
    return left >= 7000 or right >= 7000

def getNewSpeed(left, right):
    leftIndex = min(left // 1600, 10) 
    rightIndex = min(right // 1600, 10)

    leftSpeed = SPEED - MOTOR_SPEED[leftIndex]
    rightSpeed = SPEED - MOTOR_SPEED[rightIndex]

    if(leftSpeed > rightSpeed):
        rightSpeed = rightSpeed - INDEX_PENALTY[rightIndex]
    else:
        leftSpeed = leftSpeed - INDEX_PENALTY[leftIndex]

    return leftSpeed, rightSpeed

def getSensorWeigths(lsa):
    left = sum(lsa.value(i) * SENSOR_WEIGHT[i] for i in range(4))
    right = sum(lsa.value(i) * SENSOR_WEIGHT[i] for i in range(4, 8))
    return left, right

def getMidSensorWeigths(lsa):
    mid = 0
    for i in range (3,5):
        mid = mid + (lsa.value(i)*SENSOR_WEIGHT[i]) # MAX 16000
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