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

class Drive(Thread):
    def __init__(self, threadID, lsa, tank_drive, state_machine, obstacle_event):
        Thread.__init__(self)
        self.threadID = threadID
        self.lsa = lsa
        self.tank_drive = tank_drive
        self.state_machine = state_machine
        self.obstacle_event = obstacle_event  # Evento para escuchar la presencia de obstáculos
        self.error_history_size = 20
        self.error_history = deque(maxlen=self.error_history_size)

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
        # Calcula el error actual
        error = sum((i - 3.5) * black[i] for i in range(8))

        # Calcula el error promedio de las últimas n lecturas
        cumulative_error = sum(self.error_history) / len(self.error_history) if self.error_history else 0

        # Calcula el valor de steer usando los términos proporcional e integral
        is_out = (sum(black) < 1)
        if is_out:
            steer = KpOut * cumulative_error
        else:
            self.error_history.append(error)
            steer = Kp * error + Ki * cumulative_error
        
        # Si es intersección, sigue recto
        if (sum(black) >= 5):
            steer = 10.0
        
        # Ajusta las velocidades de los motores
        left_speed = SPEED - steer
        right_speed = SPEED + steer

        if self.obstacle_event.is_set():
            left_speed = left_speed * 0.05
            right_speed = right_speed * 0.05

        # Aplica los valores calculados a los motores
        self.tank_drive.on(left_speed, right_speed)

            
    
    def deliverPizza(self, black):
        if sum(black) >= 5:  # Detect bifurcation on the right side
            # Now fine tune to find 2 blacks between positions 3 and 5
            sleep(0.5)
            # TODO: no termina de girar
            self.tank_drive.on_for_seconds(5, -5, 0.6)
            self.tank_drive.on(5, -5)
            while not (black[3] and black[5]):
                sleep(0.03)
                black = self.getBlacks()
            print("Black line found, stopping adjustment.")
            self.state_machine.set_state(State.RUNNING)

    def run(self):
        print("--MOTOR--")
        self.tank_drive.on(SPEED, SPEED)
        while True:
            current_state = self.state_machine.get_state()
            black = self.getBlacks()
            
            if current_state == State.PIZZATIME:
                self.deliverPizza(black)
                self.steer(black)

            if current_state == State.RUNNING:
                self.steer(black)

            sleep(0.01)

class ColorReader(Thread):
    ROUTE_RED = [1,2,3]

    def __init__(self, threadID, sensor, state_machine):
        Thread.__init__(self)
        self.threadID = threadID
        self.sensor = sensor
        self.state_machine = state_machine
        self.recent_colors = deque(maxlen=5)
        self.traffic_light_colors = []
        self.traffic_red_pos = 0
        self.current_route_pos = 0
        self.last_trafic_read_time = 0.0

    def run(self):
        print("--COLOR SENSOR--")
        current_state = self.state_machine.get_state()
        while current_state == State.RUNNING:
            red, green, blue = self.sensor.rgb
            #print("Is red: " + str(self.isRed(red, green, blue)))
            #print("Is floor: " + str(self.isFloor(red, green, blue)))
            #print("Is pizaa time: " + str(self.isTrafficLight(red, green, blue))) 18 96 107 =>  
            if not self.isBlack(red, green, blue) and not self.isRecognizedColor(red, green, blue):
                self.add_color((red, green, blue))
            
            current_time = time()
            # Si pasan más de 2 segundos de la ultima lectura de traffic light y tiene elementos, limpiamos la lista
            if (current_time - self.last_trafic_read_time > 2) and len(self.traffic_light_colors) > 0:
                self.traffic_light_colors.clear()
                print("---TIME-OUT---")
                self.last_trafic_read_time = current_time 
            
            if self.isTrafficLight(red, green, blue):
                # self.state_machine.set_state(State.PIZZATIME)
                #  !isRecognizedColor && isTraffic => V/A, R => 2 -> 
                # [1,2,3] ; [0,0,1] ->
                self.last_trafic_read_time = current_time
                if len(self.traffic_light_colors) >= 3:
                    # miramos codigo de semaforo
                    for i in range(3):
                        r,g,b = self.traffic_light_colors[i]
                        if self.isRed(r,g,b):
                            self.traffic_red_pos = i + 1 #ajustar al array de ROUTE_RED
                            break       
                    
                    if self.ROUTE_RED[self.current_route_pos] == self.traffic_red_pos:
                        self.current_route_pos += 1
                        print("Pizza time detected, stopping drive!")
                        self.state_machine.set_state(State.PIZZATIME)

                    sleep(1.0)
                    self.traffic_light_colors.clear()

                else:
                    # si no hay -> meter el color
                    # si hay -> mirar con el anterior, si hay diferencia se mete
                    # cuando completemos mirar cuales es rojo y ponerlo asi: [0,0,1]
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
                        # print("deltas: ",delta_red, delta_green, delta_blue)
                        if delta_red > 10 or delta_green > 10 or delta_blue > 10:
                            print("[" + str(self.isRed(red, green, blue)) + "]" + "("+str(red)+", "+str(green)+", "+str(blue)+")")
                            self.traffic_light_colors.append((red, green, blue))
            sleep(0.2)

    def add_color(self, rgb):
        """Añade un nuevo color a la cola."""
        self.recent_colors.append(rgb)

    def isBlack(self, red, green, blue):
        BLACK_THRESHOLD = 20
        return abs(red-green) <= BLACK_THRESHOLD and abs(green-blue) <= BLACK_THRESHOLD

    def isRecognizedColor(self, red, green, blue):
        """Example method to check for specific recognized colors."""
        return self.isRed(red, green, blue)  # Add other recognized colors if needed
    
    def isRed(self, red, green, blue):  
        if red > 20 and red > green * 1.5 and red > blue * 1.5 :
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

class ObstacleAvoidance(Thread):
    def __init__(self, threadID, ultrasonic_sensor, tank_drive, obstacle_event):
        Thread.__init__(self)
        self.threadID = threadID
        self.ultrasonic_sensor = ultrasonic_sensor
        self.tank_drive = tank_drive
        self.obstacle_event = obstacle_event  # Evento para indicar la presencia de un obstáculo
        self.running = True

    def run(self):
        print("--DISTANCE SENSOR--")
        while self.running:
            distance = self.ultrasonic_sensor.distance_centimeters
            
            if distance < 20.0:
                # Obstáculo cercano, pero no tan cerca, aún podemos manejarlo
                self.obstacle_event.set()
            else:
                # No hay obstáculos, limpiamos el evento
                self.obstacle_event.clear()

            sleep(0.25)

if __name__ == '__main__':
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
