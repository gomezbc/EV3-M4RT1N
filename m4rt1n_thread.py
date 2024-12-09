#!/usr/bin/env python3
from ev3dev2.sensor.lego import TouchSensor, ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.sensor import Sensor, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from time import sleep
from threading import Thread, Event, Lock
from collections import deque
from enum import Enum

SPEED = 25
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
"""

class State(Enum):
    RUNNING = "running"
    PIZZATIME = "pizzatime"


class StateMachine:
    def __init__(self):
        self.state = State.RUNNING  # Estado inicial
        self.state_changed = Event()  # Señal de cambio de estado
        self.lock = Lock()

    def get_state(self):
        """Thread-safe state retrieval."""
        with self.lock:
            return self.state

    def set_state(self, new_state: State):
        """Thread-safe state modification."""
        with self.lock:
            if self.state != new_state:
                self.state = new_state
                self.state_changed.set()  # Notify threads about the state change

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

    def getBlacks(self):
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
        return black
    
    def steer(self, black):
        error = sum((i - 3.5) * black[i] for i in range(8))
        steer = Kp * error

        left_speed = SPEED - steer
        right_speed = SPEED + steer

        self.tank_drive.on(left_speed, right_speed)
            
    
    def deliverPizza(self, black):
        if sum(black) >= 5:  # Detect bifurcation on the right side
            # Now fine tune to find 2 blacks between positions 3 and 5
            sleep(0.6)
            self.tank_drive.on_for_seconds(5, -5, 1.0)
            while not (black[3] and black[5]):
                print("Fine tuning: Steering right slowly...")
                self.tank_drive.on(5, -5)
                sleep(0.05)
                black = self.getBlacks()
            print("Black line found, stopping adjustment.")
            self.state_machine.set_state(State.RUNNING)

    def run(self):
        self.tank_drive.on(SPEED, SPEED)
        while True:
            current_state = self.state_machine.get_state()
            black = self.getBlacks()
            
            if current_state == State.PIZZATIME:
                print("Pizza time detected, stopping drive!")
                self.deliverPizza(black)
                self.steer(black)

            if current_state == State.RUNNING:
                self.steer(black)

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
            red, green, blue = self.sensor.rgb
            #print("Is red: " + str(self.isRed(red, green, blue)))
            #print("Is floor: " + str(self.isFloor(red, green, blue)))
            #print("Is pizaa time: " + str(self.isTrafficLight(red, green, blue)))
            if not self.isBlack(red, green, blue) and not self.isRecognizedColor(red, green, blue):
                self.add_color((red, green, blue))
            
            if self.isTrafficLight(red, green, blue):
                self.state_machine.set_state(State.PIZZATIME)
            sleep(0.2)

    def add_color(self, rgb):
        """Añade un nuevo color a la cola."""
        self.recent_colors.append(rgb)

    def isBlack(self, red, green, blue):
        BLACK_THRESHOLD = (20, 25, 25)
        return red <= BLACK_THRESHOLD[0] and green <= BLACK_THRESHOLD[1] and blue <= BLACK_THRESHOLD[2]

    def isRecognizedColor(self, red, green, blue):
        """Example method to check for specific recognized colors."""
        return self.isRed(red, green, blue)  # Add other recognized colors if needed
    
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

        if len(self.recent_colors) == 0:
            return False

        avg_red = sum(color[0] for color in self.recent_colors) / len(self.recent_colors)
        avg_green = sum(color[1] for color in self.recent_colors) / len(self.recent_colors)
        avg_blue = sum(color[2] for color in self.recent_colors) / len(self.recent_colors)

        diff_colores = abs(blue - green) + abs(red - green)

        if avg_red >= 40 and avg_green >= 40 and avg_blue >= 40 and diff_colores <= 30:
            return True

        return False


    def isTrafficLight(self, red, green, blue):
        return not self.isFloor(red, green, blue) and not self.isBlack(red, green, blue)



if __name__ == '__main__':
    state_machine = StateMachine()
    tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)
    lsa = Sensor(INPUT_3)
    # with open("v2.log", "w") as f:
        # f.write("LOGS DE MARTIN\n")
    d = Drive(2, lsa, tank_drive, state_machine)
    d.setDaemon = True
    d.start()

    cs = ColorSensor(INPUT_4)
    cr = ColorReader(3, cs, state_machine)
    cr.start()

    #t = waitForTones(1, 1500)
    #t.setDaemon = True
    #t.start()

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
