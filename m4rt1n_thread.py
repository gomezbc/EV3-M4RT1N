#!/usr/bin/env python3
from ev3dev2.sensor.lego import TouchSensor, ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.sensor import Sensor, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from time import sleep
from threading import Thread, Event
from collections import deque
from enum import Enum

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

class State(Enum):
    RUNNING = "running"
    PIZZATIME = "pizzatime"


class StateMachine:
    def __init__(self):
        self.state = State.RUNNING  # Estado inicial
        self.state_changed = Event()  # Señal de cambio de estado

    def get_state(self):
        """Devuelve el estado actual."""
        return self.state

    def set_state(self, new_state: State):
        """Establece un nuevo estado y notifica a los hilos."""
        if self.state != new_state:
            self.state = new_state
            self.state_changed.set()  # Señaliza que el estado ha cambiado

    def wait_for_state_change(self):
        """Bloquea el hilo hasta que el estado cambie."""
        self.state_changed.wait()
        self.state_changed.clear()

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

class Drive(Thread):
    def __init__(self, threadID, lsa, tank_drive, state_machine):
        Thread.__init__(self)
        self.threadID = threadID
        self.lsa = lsa
        self.tank_drive = tank_drive
        self.state_machine = state_machine

    def run(self):
        self.tank_drive.on(SPEED, SPEED)
        while True:
            current_state = self.state_machine.get_state()

            if current_state == State.PIZZATIME:
                print("Pizza time detected, stopping drive!")
                self.tank_drive.off()  # Stop the robot
                break

            if current_state == State.RUNNING:
                lsa_values = [self.lsa.value(i) for i in range(8)]
                black_readings = [value for value in lsa_values if value < INITIAL_THRESHOLD]
                if black_readings:
                    avg_black_value = sum(black_readings) / len(black_readings)
                    threshold = avg_black_value * 1.6
                else:
                    threshold = INITIAL_THRESHOLD

                black = [0] * 8
                for i in range(8):
                    if lsa_values[i] < threshold:
                        black[i] = 1

                error = sum((i - 3.5) * black[i] for i in range(8))
                steer = Kp * error

                left_speed = SPEED - steer
                right_speed = SPEED + steer

                self.tank_drive.on(left_speed, right_speed)
            
            sleep(0.01)

class ColorReader(Thread):
    def __init__(self, threadID, sensor, state_machine):
        Thread.__init__(self)
        self.threadID = threadID
        self.sensor = sensor
        self.state_machine = state_machine
        self.recent_colors = deque(maxlen=5)

    def start(self):
        current_state = self.state_machine.get_state()
        while current_state == State.RUNNING:
            print(self.sensor.rgb)
            red = self.sensor.rgb[0]
            green = self.sensor.rgb[1]
            blue = self.sensor.rgb[2]
            #print("Is red: " + str(self.isRed(red, green, blue)))
            #print("Is floor: " + str(self.isFloor(red, green, blue)))
            #print("Is pizaa time: " + str(self.isTrafficLight(red, green, blue)))
            if self.isTrafficLight(red, green, blue):
                self.state_machine.set_state(State.PIZZATIME)
            sleep(0.2)

    def add_color(self, rgb):
        """Añade un nuevo color a la cola."""
        self.recent_colors.append(rgb)

    def isRed(self, red, green, blue):
        r_red, r_green, r_blue = 41, 12, 10
        min_intensity = 20
    
        if red > green * 1.5 and red > blue * 1.5 and red > min_intensity:
            return True
        return False
    
    def isFloor(self, red, green, blue):
        MIN_INTENSITY = 50
        if red >= MIN_INTENSITY and green >= MIN_INTENSITY and blue >= MIN_INTENSITY:
            return True
        
        return False


    def isTrafficLight(self, red, green, blue):
        BLACK_TH = (20,25,25)

        if red <= BLACK_TH[0] and green <= BLACK_TH[1] and blue <= BLACK_TH[2]:
            return False
        
        return not self.isFloor(red, green, blue)


if __name__ == '__main__':
    state_machine = StateMachine()
    tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)
    lsa = Sensor(INPUT_3)
    with open("v2.log", "w") as f:
        f.write("LOGS DE MARTIN\n")
    d = Drive(2, lsa, tank_drive, state_machine)
    d.setDaemon = True
    d.start()

    cs = ColorSensor(INPUT_4)
    cr = ColorReader(3, cs, state_machine)
    cr.start()

    t = waitForTones(1, 1500)
    t.setDaemon = True
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
