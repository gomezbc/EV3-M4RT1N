#!/usr/bin/env python3

from ev3dev2.sensor import Sensor, INPUT_3
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from time import sleep

SPEED = 30
INITIAL_THRESHOLD = 70
Kp = 1.0 

def main():
    tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)
    lsa = Sensor(INPUT_3)
    tank_drive.on(SPEED, SPEED)

    while True:
        lsa_values = [lsa.value(i) for i in range(8)]

        # Filter out black readings using the initial threshold
        black_readings = [value for value in lsa_values if value < INITIAL_THRESHOLD]

        # Calculate the average of the black readings
        if black_readings:
            avg_black_value = sum(black_readings) / len(black_readings)
            # Set the threshold slightly below the average black value
            threshold = avg_black_value * 0.8
        else:
            threshold = INITIAL_THRESHOLD

        black = [0, 0, 0, 0, 0, 0, 0, 0]
        
        for i in range(8):
            if lsa_values[i] < threshold:
                black[i] = 1
        
        # Calculate error as the difference between the center and the detected black line
        error = sum((i - 3.5) * black[i] for i in range(8))
        
        # Proportional control
        steer = Kp * error
        
        # Adjust motor speeds based on the steer value
        left_speed = SPEED - steer
        right_speed = SPEED + steer

        tank_drive.on(left_speed, right_speed)
        
        sleep(0.01)


if __name__ == "__main__":
    main()
