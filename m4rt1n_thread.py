#!/usr/bin/env python3
from ev3dev2.sensor.lego import TouchSensor, ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.sensor import Sensor, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from time import sleep
from threading import Thread, Event
from collections import deque

SPEED = 30
INITIAL_THRESHOLD = 70
Kp = 1.2

# crear una maquina de estados
"""
TODO:
+ MAQUINA DE ESTADOS: CIRCULAR (LEER) -> REPARTIR(MODO GIRO) -
+ SI ES EL COLOR QUE QUEREMOS REPARTIR ENTRA EN MODO GIRO 
    + MODO GIRO => QUE CUANDO ESTE EN ESTE MODO VA A MIRAR LOS NEGROS LATERALES DERECHOS SI HAY BIFURCACIÓN
    + LA COSA ES QUE HAY QUE MIRARLA TODO EL RATO, 
    1. MIRAMOS BIFURCAS TODORA (METEMOS UNA PILA, LISTA ... CON LOS ÚLTIMOS RELECANTES (4?))
    2. SI ES BIFURCA DERECHA, SINO RECTO

    SUELO R:60 G:60 B:50
    V1 R:40 G:94 B:70 
    A1 R:15 G:74 B:82 
    R1 R:41 G:12 B:10
    NEGRO1: R10 G17 B15 
"""

class waitForTones(Thread):
    def __init__(self, threadID, frequency):
        Thread.__init__(self)
        self.threadID = threadID
        self.sound = Sound()
        self.running = True
        self.frequency = frequency

    def run(self):
        while self.running:
            for j in range(0,20):             # Do twenty times.
                self.sound.play_tone(self.frequency, 0.2) #1500Hz for 0.2s
                sleep(0.5)
        self.sound.beep()

class Drive(Thread):
    def __init__(self, threadID, lsa, tank_drive, stop_event):
        Thread.__init__(self)
        self.threadID = threadID
        self.lsa = lsa
        self.tank_drive = tank_drive
        self.running = True
        self.stop_event = stop_event

    def run(self):
        self.tank_drive.on(SPEED, SPEED)
        while self.running:
            # Check if the stop event is set
            if self.stop_event.is_set():
                self.tank_drive.off()
                print("Stopping drive: It's pizza time!")
                break

            # Main driving logic
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

            self.tank_drive.on(left_speed, right_speed)
            
            sleep(0.01)

class ColorReader(Thread):
    def __init__(self, threadID, sensor, stop_event):
        Thread.__init__(self)
        self.threadID = threadID
        self.sensor = sensor
        self.reading = True
        self.lastReads = deque(maxlen=4)
        self.stop_event = stop_event

    def run(self):
        # print("ColorReader started")
        while self.reading:
            if self.sensor.rgb is None:
                # print("Error: Sensor de color no está funcionando correctamente.")
                continue
            red, green, blue = self.sensor.rgb
            # print("Red: " + str(red) + ", Green: " + str(green) + ", Blue: " + str(blue))
            if self.isPizzaTime(red, green, blue):
                # print("Pizza time detected! Signaling to stop driving.")
                self.stop_event.set()  # Signal the event
                self.reading = False  # Stop reading
            sleep(0.3)
                
    def isPizzaTime(self, red, green, blue):
        isPizzaTime = False
        if len(self.lastReads) == 4:
            avg_r, avg_g, avg_b = self.calculate_average()
            diff_r, diff_g, diff_b = self.calculate_diff((red, green, blue), (avg_r, avg_g, avg_b))
            diff_sum = diff_r + diff_g + diff_b
            print(diff_sum)
            isPizzaTime =  diff_sum >= 20
        self.add_reading(red, green, blue)
        print(isPizzaTime)
        return isPizzaTime


    def add_reading(self, red, green, blue):
        """Add a new RGB reading to the queue."""
        self.lastReads.append((red, green, blue))

    def calculate_average(self):
        """Calculate the average RGB values from the queue."""
        if not self.lastReads:
            return (0, 0, 0)  # Return (0, 0, 0) if the queue is empty
        
        red_total = green_total = blue_total = 0
        for r, g, b in self.lastReads:
            red_total += r
            green_total += g
            blue_total += b
        
        count = len(self.lastReads)
        return (
            red_total / count,
            green_total / count,
            blue_total / count
        )
    
    def calculate_diff(self, rgb, avg_rgb):
        """Calculate the absolute difference between two RGB values."""
        r, g, b = rgb
        avg_r, avg_g, avg_b = avg_rgb
        return (
            abs(avg_r - r),
            abs(avg_g - g),
            abs(avg_b - b)
        )



if __name__ == '__main__':
    tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)
    lsa = Sensor(INPUT_3)
    stop_event = Event()  # Create the shared event
    with open("v2.log", "w") as f:
        f.write("LOGS DE MARTIN\n")

    d = Drive(2, lsa, tank_drive, stop_event)
    d.daemon = True
    d.start()

    cs = ColorSensor(INPUT_4)
    cr = ColorReader(3, cs, stop_event)
    cr.daemon = True
    cr.start()

    """
    t = waitForTones(1, 1500)
    t.daemon = True
    t.start()

    ts = TouchSensor(INPUT_2)
    running = True
    counter = 0
    while running:
        ts.wait_for_bump()
        counter = counter + 1
        if counter >=5:
            t.running = False
            running = False
        sleep(0.01)
    """
