#!/usr/bin/env python3

from ev3dev2.sensor import Sensor, INPUT_3
from ev3dev2.motor import MoveTank, OUTPUT_C, OUTPUT_A 

from time import sleep
SPEED = 15
LIGHT_TARGET_WEIGHT = 10
Kp = 0.6

LOGS_FILE = 'logs.txt'

def writeInfo(light_weight):
    with open(LOGS_FILE, 'a') as f:
        f.write("light_weight: " + str(light_weight) + "\n")

def hasToSteer(light_weight):
    writeInfo(light_weight)
    # print to see how much is light_weight and change 4
    # it has to be a value low enought to not steer whe is going fordward, and to start steering when it should
    return LIGHT_TARGET_WEIGHT - light_weight > 4

def getNewSpeed(light_weight):
    error = LIGHT_TARGET_WEIGHT - light_weight

    correction = Kp * error

    left_speed = base_speed + correction
    right_speed = base_speed - correction
    return left_speed, right_speed



def getLightWeigth(lsa):
    sensor_values = [lsa.value(i) for i in range(0,8)]
    weighted_sum = sum(i * sensor_values[i] for i in range(8))
    total_sum = sum(sensor_values)
    
    if total_sum == 0:
        return 0
    # it returns how much bigger is the weighted_sum compared to total_sum
    return weighted_sum / total_sum

def main():
    tank_drive = MoveTank(OUTPUT_A, OUTPUT_C)
    tank_drive.on(SPEED,SPEED)
    lsa = Sensor(INPUT_3)
    i=0
    while True:
        light_weight = getLightWeigth(lsa)
        if hasToSteer(light_weight):
           speedLeft, speedRight = getNewSpeed(left, right)
           tank_drive.on(speedLeft, speedRight)
        else:
            tank_drive.on(SPEED,SPEED)
        i += 1
        sleep(0.05)

if __name__ == "__main__":
    main()
