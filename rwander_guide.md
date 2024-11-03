rwander.py fitxategian hainbat argumentu declaratzen ditu, vel_topic, scan_topic eta pose_topic.

2 subscriber eta 6 publisher

Lidera 0 tik 1600 doa. 0 delante (-pi), 400 izquierda (-pi/2), 800 atras(0), 1200 derecha(pi/2) y 1600 delante (pi). Erabili balore hauek angelua neurtzeko.

bearings erakurketak dira eta scan aktualizatu behar dugu


/cmd.vel topikean idazten diren mesuak geometry_msgs/msg/Twist motakoak dira

/scan topikean, sensor_msgs/msg/LaserScan

TODO:
self._scan taula procesatu ostopoak dauden jakiteko eta horren arabera abiadura eta abiadura angeluarra ezarri


TIPS:
laserra hiru "quesitotan" banatu, L, F eta R. 

R = sumatorio quesitoaren valore gustiari / valore kopurua

for(i=0, i < nr, i++):
    sum += sensor[i]

R = sum / nr

if L < R : w = -0.2
if L > R : w = 0.2
if F < Th: V = 0.0
else: v = 0.2

Bataz bestekoa erabiliz errasa da bahino es da hoberena.

Hiru if bainu gehiago antikristoa da. 


Marra beltza segitzeko:
Gure helburua: erdiko tartea erdian egotea eta muturra erdian mantentzen sahiatzea

Ezarri threshold bat balore txiak ignoratzeko
