#!/usr/bin/env python3

from time import sleep
import math
from smbus import SMBus
from ev3dev2.sound import Sound
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveDifferential, MoveTank, SpeedRPM
from ev3dev2.port import LegoPort
from ev3dev2.wheel import EV3Tire


# multi threading complet
# check la balle avec un demi cercle puis le pattern
# check la couleur et la taille de la balle ou la forme. Eviter les mauvais objectifs
# code python créer un xml --> envoyer en php sur une page web


def limit_speed(speed):
    """ Limit speed in range [-1000,1000] """
    if speed > 1000:
        speed = 1000
    elif speed < -1000:
        speed = -1000
    return speed

# Set LEGO port for Pixy2 on input port 1
in1 = LegoPort(INPUT_1)
in1.mode = 'other-i2c'
# Short wait for port to get ready
sleep(0.5)

# Settings for I2C (SMBus(3) for INPUT_1)
bus = SMBus(3)
# Make sure the same address is set in Pixy2
address = 0x54

# Signatures we're interested in (SIG1)
sig = 1

# Connect TouchSensor (to stop script)
ts = TouchSensor(INPUT_4)
spkr = Sound()
spkr.set_volume(100)
# Connect LargeMotors
rmotor = LargeMotor(OUTPUT_B)
lmotor = LargeMotor(OUTPUT_A)
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
grab = LargeMotor(OUTPUT_C)
calibrateRotation = 94 #100 avec roues bien propre sur parquet (ajuster si besoin)
mdiff = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, calibrateRotation)

# Defining constants
X_REF = 158  # X-coordinate of referencepoint
Y_REF = 150  # Y-coordinate of referencepoint
KP = 0.1     # Proportional constant PID-controller
KI = 0.01    # Integral constant PID-controller
KD = 0.05    # Derivative constant PID-controller
GAIN = 5    # Gain for motorspeed

# Initializing PID variables
integral_x = 0
derivative_x = 0
last_dx = 0
integral_y = 0
derivative_y = 0
last_dy = 0

#Definition on robot's specifications
wheelDiam = 5.5
wheelRayon = wheelDiam/2
wheelPerimeter = 2*(math.pi)*wheelRayon

#sur piximon, calibrer la balle à environ 1.5 telephone sans flash
# Data for requesting block
data = [174, 193, 32, 2, sig, 1]
#Side length en cm
coeff = 10.5
sideLength = 100
sideOdo = coeff * sideLength
sidePart = sideOdo/4
speed = SpeedRPM(40)

#parcours 1/2
#X = [0,0,2*sidePart,2*sidePart,sideOdo,sideOdo]
#Y = [0,sideOdo,sideOdo,0,0,sideOdo]

#parcours 1/3
#X = [0  ,0        ,sideOdo/3  ,sideOdo/3  ,2*(sideOdo/3)  ,2*(sideOdo/3)  ,sideOdo  ,sideOdo]
#Y = [0  ,sideOdo  ,sideOdo    ,0          ,0              ,sideOdo        ,sideOdo  ,0]

#parcours 1/4
X = [0  ,0        ,sidePart  ,sidePart  ,2*sidePart  ,2*sidePart  ,3*sidePart  ,3*sidePart  ,sideOdo  ,sideOdo]
Y = [0  ,sideOdo  ,sideOdo   ,0         ,0           ,sideOdo     ,sideOdo     ,0           ,0        ,sideOdo] 

def linspace(a,b,pas):
    coeff = 10.5
    L = []
    nbr=int(abs((b-a)/(coeff*pas)))
    if b-a < 0:
        for i in range(nbr):
            valeur = a - (i+1) * coeff*pas
            L.append(valeur)
    else:
        for i in range(nbr):
            valeur = a + (i+1) * coeff*pas
            L.append(valeur)
    return(L)

def check(integral_x,derivative_x,last_dx,integral_y,derivative_y,last_dy,xLast,yLast,index):
    time = 0
    detectTime = 0
    nbrRotation = 0
    lastTraject = 0
    arrive = 0
    while not ts.value():
        # Request block
        bus.write_i2c_block_data(address, 0, data)
        # Read block
        block = bus.read_i2c_block_data(address, 0, 20)
        if sig == block[7]*256 + block[6]:
            print("detect")
            # SIG1 detected, control motors
            x = block[9]*256 + block[8]   # X-centroid of largest SIG1-object
            y = block[11]*256 + block[10] # Y-centroid of largest SIG1-object
            dx = X_REF - x                # Error in reference to X_REF
            integral_x = integral_x + dx  # Calculate integral for PID
            derivative_x = dx - last_dx   # Calculate derivative for PID
            speed_x = KP*dx + KI*integral_x + KD*derivative_x  # Speed X-direction
            dy = Y_REF - y                # Error in reference to Y_REF
            integral_y = integral_y + dy  # Calculate integral for PID
            derivative_y = dy - last_dy   # Calculate derivative for PID
            speed_y = KP*dy + KI*integral_y + KD*derivative_y  # Speed Y-direction
            # Calculate motorspeed out of speed_x and speed_y
            # Use GAIN otherwise speed will be to slow,
            # but limit in range [-1000,1000]
            rspeed = limit_speed(GAIN*(speed_y - speed_x))
            lspeed = limit_speed(GAIN*(speed_y + speed_x))
            rmotor.run_forever(speed_sp = round(rspeed))
            lmotor.run_forever(speed_sp = round(lspeed))
            last_dx = dx                  # Set last error for x
            last_dy = dy                  # Set last error for y
            detectTime = detectTime + 0.1
            if detectTime > 5:
                rmotor.stop()
                lmotor.stop()
                while arrive == 0:
                    # Request block
                    bus.write_i2c_block_data(address, 0, data)
                    # Read block
                    block = bus.read_i2c_block_data(address, 0, 20)
                    if sig == block[7]*256 + block[6]:
                        print('je te vois toujours')
                        tank_drive.on_for_rotations(SpeedPercent(20),SpeedPercent(20), 1)
                        nbrRotation = nbrRotation + 1
                        lastTraject = lastTraject + wheelPerimeter
                    else:
                        grab.on_for_rotations(SpeedRPM(30), 0.15)
                        print("je rentre a la maison")
                        #adjacent = math.cos(40) * lastTraject
                        #oppose = math.sin(40) * lastTraject
                        #xf = xLast + adjacent
                        #yf = yLast + oppose
                        spkr.play_song((
                            ('C5', 'e*0.1'),
                            ('C#5', 'e*0.1'),
                            ('D5', 'e*0.1'),
                            ('D#5', 'e*0.1'),
                            ('E5', 'e*0.1'),
                        ))
                        print(nbrRotation)
                        tank_drive.on_for_rotations(SpeedPercent(20),SpeedPercent(20), -nbrRotation)
                        if index == 1 or index == 5 or index == 9:
                            #x = xf
                            #y = yf - yLast
                            mdiff.turn_to_angle(speed, 90)
                        elif index == 2 or index == 4 or index == 6 or index == 8:
                            #x = xLast
                            #y = yf - yLast
                            mdiff.turn_to_angle(speed, 0)
                        else : 
                            #x = xf
                            #y = yf
                            mdiff.turn_to_angle(speed, 270)
                        #return mdiff.on_to_coordinates(speed, x - (wheelPerimeter), y - (wheelPerimeter))
                        return mdiff.on_to_coordinates(speed, 0, 0)
        else:
            # SIG1 not detected, stop motors
            rmotor.stop()
            lmotor.stop()
            last_dx = 0
            last_dy = 0
            time = time + 0.1
            print(time)
        if time > 12:
            rmotor.stop()
            lmotor.stop()
            adjacent = math.cos(10) * lastTraject
            oppose = math.sin(10) * lastTraject
            xf = xLast + adjacent
            yf = yLast + oppose
            grab.on_for_rotations(SpeedRPM(30), 0.22)
            print("je rentre a la maison")
            backHome = 1
            return mdiff.on_to_coordinates(speed, xf, yf-yLast)

def trajet(index,p1,p2):
    global backHome 
    backHome = 0
    while backHome == 0:
        # Request block
        bus.write_i2c_block_data(address, 0, data)
        # Read block
        block = bus.read_i2c_block_data(address, 0, 20)
        x1,y1=p1
        x2,y2=p2
        if index%2 == 0:
            print("pair")
            longueur = linspace(x1,x2,10)
            print(longueur)
            for j in longueur:
                # Request block
                bus.write_i2c_block_data(address, 0, data)
                # Read block
                block = bus.read_i2c_block_data(address, 0, 20)
                if sig == block[7]*256 + block[6]:
                    backHome = 1
                    return check(0,0,0,0,0,0,j/10.5,y1, index)
                else:
                    mdiff.on_to_coordinates(speed, j, y1)
            break
        else:
            print("impair")
            longueur = linspace(y1,y2,10)
            print(longueur)
            for j in longueur:
                # Request block
                bus.write_i2c_block_data(address, 0, data)
                # Read block
                block = bus.read_i2c_block_data(address, 0, 20)
                if sig == block[7]*256 + block[6]:
                    backHome = 1
                    return check(0,0,0,0,0,0
                    ,x1,j/10.5, index)
                else:
                    mdiff.on_to_coordinates(speed, x1, j)
            break

def square():
    mdiff.on_to_coordinates(speed, 0, sideOdo/3)
    mdiff.on_to_coordinates(speed, sideOdo/3, sideOdo/3)
    mdiff.on_to_coordinates(speed, sideOdo/3, 0)
    mdiff.on_to_coordinates(speed, 0, 0)
    mdiff.turn_to_angle(speed, 90)


print('robot is ready')
spkr.speak('Lets go')

mdiff.odometry_start(theta_degrees_start=90.0, x_pos_start=0.0, y_pos_start=0.0, sleep_time=0.005)

for i in range(len(X)):
    print(i)
    p1 = (X[i],Y[i])
    p2 = (X[i+1],Y[i+1])
    trajet(i+1,p1,p2)
    print(backHome)
    if backHome == 1:
        break


#square()
mdiff.turn_to_angle(speed, 90)
rmotor.stop()
lmotor.stop()

print("fin")

