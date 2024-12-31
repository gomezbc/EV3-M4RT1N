#!/usr/bin/env python3
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor
from ev3dev2.sound import Sound
from ev3dev2.sensor import Sensor, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from time import sleep, time
from threading import Thread, Event, Lock
from collections import deque
from enum import Enum

SPEED = 25
INITIAL_THRESHOLD = 70
Kp = 1.2
KpOut = 5.0
Ki = 0.12

class State(Enum):
    RUNNING = "running"  # Ohiko ibilbidea
    PIZZATIME = "pizzatime"  # Pizza banatzeko modua

class StateMachine:
    """
    Egoera-makina kudeatzeko klasea:
    - Robotaren egoera gordetzeko
    - Egoera aldaketak hari-seguru moduan kudeatzeko
    - Beste hariak egoera aldaketen zain egoteko
    """
    def __init__(self):
        self.state = State.RUNNING
        self.state_changed = Event()
        self.lock = Lock()

    def get_state(self):
        """Hari-segurua den egoeraren lorpena."""
        with self.lock:
            return self.state

    def set_state(self, new_state: State):
        """Hari-segurua den egoeraren aldaketa."""
        with self.lock:
            if self.state != new_state:
                self.state = new_state
                self.state_changed.set()

    def wait_for_state_change(self):
        """Haria blokeatu egoera aldatu arte."""
        self.state_changed.wait()
        self.state_changed.clear()

class Drive(Thread):
    """
    Robotaren mugimendua kudeatzen duen haria:
    - Lerro beltzaren jarraipena egiten du
    - Bidegurutzeak detektatzen ditu
    - Pizza banatzeko biraketa kudeatzen du
    - Oztopoak daudenean abiadura doitzen du
    """
    def __init__(self, threadID, lsa, tank_drive, state_machine, obstacle_event):
        Thread.__init__(self)
        self.threadID = threadID
        self.lsa = lsa
        self.tank_drive = tank_drive
        self.state_machine = state_machine
        self.obstacle_event = obstacle_event
        self.error_history_size = 20
        self.error_history = deque(maxlen=self.error_history_size)

    def getBlacks(self):
        """
        Lerro beltzak detektatzeko funtzioa:
        1. 8 sentsoreen balioak irakurtzen ditu
        2. Atalase dinamikoa kalkulatzen du beltzen irakurketekin
        3. Atalase hori erabiliz, 0/1 array bat itzultzen du
        """
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
        """
        Robotaren norabidea kalkulatu eta aplikatzen duen funtzioa:
        1. Uneko errorea kalkulatzen du lerro beltzarekiko
        2. Errore metatua kalkulatzen du azken N irakurketekin
        3. PI kontrola aplikatzen du
        4. Bidegurutzeetan zuzen joateko logika aplikatzen du
        5. Oztopoak daudenean abiadura murrizten du
        """
        error = sum((i - 3.5) * black[i] for i in range(8))
        cumulative_error = sum(self.error_history) / len(self.error_history) if self.error_history else 0

        is_out = (sum(black) < 1)
        if is_out:
            steer = KpOut * cumulative_error
        else:
            self.error_history.append(error)
            steer = Kp * error + Ki * cumulative_error
        
        if (sum(black) >= 5):
            steer = 10.0
        
        left_speed = SPEED - steer
        right_speed = SPEED + steer

        if self.obstacle_event.is_set():
            left_speed = left_speed * 0.05
            right_speed = right_speed * 0.05

        self.tank_drive.on(left_speed, right_speed)
            
    def deliverPizza(self, black):
        """
        Pizza banatzeko biraketa kudeatzen duen funtzioa:
        1. Bidegurutzea detektatzen du (>= 5 sentsore beltz)
        2. Biraketa hasten du eskuinerantz
        3. Lerro beltza aurkitu arte biratzen jarraitzen du
        4. Running egoerara itzultzen da
        """
        if sum(black) >= 5:
            sleep(0.5)
            self.tank_drive.on_for_seconds(5, -5, 0.6)
            self.tank_drive.on(5, -5)
            while not (black[3] and black[5]):
                sleep(0.03)
                black = self.getBlacks()
            self.state_machine.set_state(State.RUNNING)

class ColorReader(Thread):
    """
    Koloreen irakurketa kudeatzen duen haria:
    - Semaforo sekuentziak detektatzen ditu
    - Kolore gorriak identifikatzen ditu
    - Lurra eta beste kolore ezagunak bereizten ditu
    - Pizza banatzeko unea erabakitzen du sekuentziaren arabera
    """
    ROUTE_RED = [1,2,3]

    def run(self):
        """
        Kolore irakurketa kudeatzeko funtzio nagusia:
        1. RGB balioak irakurtzen ditu
        2. Kolore ezagunak eta beltzak baztertzen ditu
        3. Semaforoen sekuentziak prozesatzen ditu
        4. Denbora-muga aplikatzen du sekuentzietarako
        5. Pizza banatzeko unea erabakitzen du
        """
        print("--COLOR SENSOR--")
        current_state = self.state_machine.get_state()
        while current_state == State.RUNNING:
            red, green, blue = self.sensor.rgb
            
            if not self.isBlack(red, green, blue) and not self.isRecognizedColor(red, green, blue):
                self.add_color((red, green, blue))
            
            current_time = time()
            if (current_time - self.last_trafic_read_time > 2) and len(self.traffic_light_colors) > 0:
                self.traffic_light_colors.clear()
                print("---TIME-OUT---")
                self.last_trafic_read_time = current_time 
            
            if self.isTrafficLight(red, green, blue):
                self.last_trafic_read_time = current_time
                if len(self.traffic_light_colors) >= 3:
                    for i in range(3):
                        r,g,b = self.traffic_light_colors[i]
                        if self.isRed(r,g,b):
                            self.traffic_red_pos = i + 1
                            break       
                    
                    if self.ROUTE_RED[self.current_route_pos] == self.traffic_red_pos:
                        self.current_route_pos += 1
                        print("Pizza time detected, stopping drive!")
                        self.state_machine.set_state(State.PIZZATIME)

                    sleep(1.0)
                    self.traffic_light_colors.clear()
                else:
                    if len(self.traffic_light_colors) == 0:
                        print("------Traffic Light-------")
                        print("[" + str(self.isRed(red, green, blue)) + "]"+ "("+str(red)+", "+str(green)+", "+str(blue)+")")
                        self.traffic_light_colors.append((red, green, blue))
                    else:
                        past_red = self.traffic_light_colors[-1][0]
                        past_green = self.traffic_light_colors[-1][1]
                        past_blue = self.traffic_light_colors[-1][2]
                        delta_red = abs(red - past_red)
                        delta_green = abs(green - past_green)
                        delta_blue = abs(blue - past_blue)
                        if delta_red > 10 or delta_green > 10 or delta_blue > 10:
                            print("[" + str(self.isRed(red, green, blue)) + "]" + "("+str(red)+", "+str(green)+", "+str(blue)+")")
                            self.traffic_light_colors.append((red, green, blue))
            sleep(0.2)

    def isFloor(self, red, green, blue):
        """
        Lurra detektatzeko funtzioa:
        1. Intentsitate minimoa egiaztatzen du
        2. Azken koloreen batezbestekoa kalkulatzen du
        3. Koloreen arteko diferentzia txikia dela egiaztatzen du
        """
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

class ObstacleAvoidance(Thread):
    """
    Oztopoak detektatu eta saihesteko haria:
    - Distantzia sentsorea erabiltzen du
    - Gertuko oztopoak detektatzen ditu (<20cm)
    - Obstacle event-a aktibatzen du oztopoak daudenean
    """
    def run(self):
        print("--DISTANCE SENSOR--")
        while self.running:
            distance = self.ultrasonic_sensor.distance_centimeters
            
            if distance < 20.0:
                self.obstacle_event.set()
            else:
                self.obstacle_event.clear()

            sleep(0.25)

if __name__ == '__main__':
    # Hasieraketak eta hariak martxan jarri
    state_machine = StateMachine()
    tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)
    lsa = Sensor(INPUT_3)

    us = UltrasonicSensor(INPUT_2)
    obstacle_event = Event() 
    oa = ObstacleAvoidance(4, us, tank_drive, obstacle_event)
    oa.setDaemon = True
    oa.start()

    d = Drive(2, lsa, tank_drive, state_machine, obstacle_event)
    d.setDaemon = True
    d.start()

    cs = ColorSensor(INPUT_4)
    cr = ColorReader(3, cs, state_machine)
    cr.setDaemon = True
    cr.start()
