#!/usr/bin/env python3
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.sound import Sound
from ev3dev2.sensor import Sensor, INPUT_3, INPUT_4
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from time import sleep
from threading import Thread

SPEED = 30
INITIAL_THRESHOLD = 70
Kp = 1.2

# crear una maquina de estados

class waitForTones(Thread):
    def __init__(self, threadID, frequency):
        Thread.__init__(self)
        self.threadID = threadID
        self.sound = Sound()
        self.running = True

    def run(self):
        while self.running:
            for j in range(0,20):             # Do twenty times.
                self.sound.play_tone(self.frequency, 0.2) #1500Hz for 0.2s
                sleep(0.5)
        self.sound.beep()

class drive(Thread):
    def __init__(self, threadID, lsa, tank_drive):
        Thread.__init__(self)
        self.threadID = threadID
        self.lsa = lsa
        self.tank_drive = tank_drive
        self.running = True

    def run(self):
        tank_drive.on(SPEED, SPEED)
        while self.running:
            lsa_values = [lsa.value(i) for i in range(8)]

        # Filter out black readings using the initial threshold
        black_readings = [value for value in lsa_values if value < INITIAL_THRESHOLD]

        # Calculate the average of the black readings
        if black_readings:
            avg_black_value = sum(black_readings) / len(black_readings)
            # Set the threshold slightly below the average black value
            threshold = avg_black_value * 1.6
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
        with open("v2.log", "a") as f:
            f.write("kp(%s): error: %s  | steer: %s |  blacks: %s  | th: %s\n" % (str(Kp), str(error), str(steer), str(black), str(threshold)))
        
        # Adjust motor speeds based on the steer value
        left_speed = SPEED - steer
        right_speed = SPEED + steer

        tank_drive.on(left_speed, right_speed)
        
        sleep(0.01)


if __name__ == '__main__':
    tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)
    lsa = Sensor(INPUT_3)
    with open("v2.log", "w") as f:
        f.write("LOGS DE MARTIN\n")
    d = drive(2, lsa, tank_drive)
    d.setDaemon = True
    d.start()
    t = waitForTones(1, 1500)
    t.setDaemon = True
    t.start()
    ts = TouchSensor()
    running = True
    counter = 0
    while running:
        ts.wait_for_bump()
        counter = counter + 1
        if counter >=5:
            t.running = False
            running = False
        sleep(0.01)
