#!/usr/bin/python3

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, MoveDifferential, SpeedRPM, SpeedPercent, MoveTank, SpeedDPS
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev2.port import LegoPort
from ev3dev2.wheel import EV3Tire
import math
import random
import time

#Definition of robot's inputs and outputs

# Set LEGO port for Pixy on input port 1
#in1 = LegoPort(INPUT_2)
#in1.mode = 'auto'
# Wait 2 secs for the port to get ready


# Connect Pixy camera
#pixy = Sensor(INPUT_2)
# Set mode to detect signature 1 only
#pixy.mode = 'SIG1'

m1 = LargeMotor(OUTPUT_A)
m2 = LargeMotor(OUTPUT_B)
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
ts = TouchSensor(INPUT_1)
leds = Leds()
#mdiff = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, 67)
mdiff = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, 94)

print('Robot Starting')

#Definition on robot's specifications

wheelDiam = 5.5
wheelRayon = wheelDiam/2
wheelPerimeter = 2*(math.pi)*wheelRayon
#Side length en cm
sideLength = 100
#Nb of rotations needed to run trough side length
nbRotationSL = sideLength/wheelPerimeter



def turnLeft():
    m1.on_for_rotations(SpeedPercent(20), 0.8)

def turnRight():
    m2.on_for_rotations(SpeedPercent(20), 0.8)

#---------------------------------------------------------------
# For different side length pattern change "sideLength" parameter, 
# the following functions are adaptative
#---------------------------------------------------------------

#Drive for side length
def move_forward():
    tank_drive.on_for_rotations(SpeedPercent(30),SpeedPercent(30), nbRotationSL)

#Drive on side for side lenght/4
def sideDriving():
    tank_drive.on_for_rotations(SpeedPercent(30),SpeedPercent(30), nbRotationSL/4)

#Make a square
def square():
    move_forward()
    turnLeft()
    move_forward()
    turnLeft()
    move_forward()
    turnLeft()
    move_forward()
    turnLeft()

def pattern1():
    for i in range(2):
        move_forward()
        turnLeft()
        sideDriving()
        turnLeft()
        move_forward()
        turnRight()
        sideDriving()
        turnRight()

    move_forward()
    turnRight()
    move_forward()
    turnRight()
    move_forward()
    turnRight()
    turnRight()

def pattern1Odo():
    sideOdo = 10.5 * sideLength
    sidePart = sideOdo/4
    speed = SpeedRPM(40)
    mdiff.on_to_coordinates(speed, 0, sideOdo)
    mdiff.on_to_coordinates(speed, sidePart, sideOdo)
    mdiff.on_to_coordinates(speed, sidePart, 0)
    mdiff.on_to_coordinates(speed, 2*sidePart, 0)
    mdiff.on_to_coordinates(speed, 2*sidePart, sideOdo)
    mdiff.on_to_coordinates(speed, 3*sidePart, sideOdo)
    mdiff.on_to_coordinates(speed, 3*sidePart, 0)
    mdiff.on_to_coordinates(speed, 4*sidePart, 0)

    mdiff.on_to_coordinates(speed, 4*sidePart, sideOdo)
    mdiff.on_to_coordinates(speed, 0, sideOdo)
    mdiff.on_to_coordinates(speed, 0, 0)
    mdiff.turn_to_angle(SpeedRPM(20), 90)

def pattern2Odo():
    sideOdo = 10.5 * sideLength
    sidePart = sideOdo/4
    speed = SpeedRPM(40)
    speed2 = SpeedRPM(40)
    mdiff.on_to_coordinates(speed, sideOdo/4, sideOdo/4)
    mdiff.turn_degrees(speed2,360)
    mdiff.on_to_coordinates(speed, sideOdo/4, 3*(sideOdo/4))
    mdiff.turn_degrees(speed2,360)
    mdiff.on_to_coordinates(speed, 3*(sideOdo/4), 3*(sideOdo/4))
    mdiff.turn_degrees(speed2,360)
    mdiff.on_to_coordinates(speed, 3*(sideOdo/4), sideOdo/4)
    mdiff.turn_degrees(speed2,360)
    mdiff.on_to_coordinates(speed, 0, 0)

def pattern3Odo():
    sideOdo = 10.5 * sideLength
    sidePart = sideOdo/4
    speed = SpeedRPM(40)
    speed2 = SpeedRPM(40)
    mdiff.on_to_coordinates(speed, sideOdo/2, sideOdo/2)
    mdiff.turn_degrees(speed2,360)
    mdiff.on_to_coordinates(speed, 0, 0)

def randomBackHome():

    tab = []
    time.sleep(2)
    i = 1
    nbMoove = 5

    while i <= nbMoove:

        rotation = random.randint(1, 3)
        speed1 = random.randint(10, 30)
        speed2 = random.randint(10, 30)
        tank_drive.on_for_rotations(SpeedPercent(speed1),SpeedPercent(speed2), rotation)
        tab.append([speed1,speed2,rotation])
        i+= 1

    #j = nbMoove
    #while j > 0:
    #    tank_drive.on_for_rotations(SpeedPercent(-(tab[j-1][0])),SpeedPercent(-(tab[j-1][1])), tab[j-1][2])
    #    j-= 1

    print(tab)
    print(tab[0])
    print(tab[0][0])

def path2():
    mdiff.on_to_coordinates(SpeedRPM(vitesse), 0, 1000)
    mdiff.on_to_coordinates(SpeedRPM(vitesse), 200, 1000)
    mdiff.on_to_coordinates(SpeedRPM(vitesse), 200, 0)
    mdiff.on_to_coordinates(SpeedRPM(vitesse), 400, 0)
    mdiff.on_to_coordinates(SpeedRPM(vitesse), 400, 1000)
    mdiff.on_to_coordinates(SpeedRPM(vitesse), 600, 1000)
    mdiff.on_to_coordinates(SpeedRPM(vitesse), 600, 0)
    mdiff.on_to_coordinates(SpeedRPM(vitesse), 800, 0)
    mdiff.on_to_coordinates(SpeedRPM(vitesse), 800, 1000)
    mdiff.on_to_coordinates(SpeedRPM(vitesse), 1000, 1000)
    mdiff.on_to_coordinates(SpeedRPM(vitesse), 1000, 0)
    mdiff.on_to_coordinates(SpeedRPM(vitesse), 0, 0)

if ts.wait_for_pressed():

    print('btn pressed')
    time.sleep(2)
    # Enable odometry
    mdiff.odometry_start(theta_degrees_start=90.0, x_pos_start=0.0, y_pos_start=0.0, sleep_time=0.005)
    pattern3Odo()

    # Use odometry to drive to specific coordinates
    #mdiff.on_to_coordinates(SpeedRPM(30), 0, 0)
    # Use odometry to rotate in place to 90 degrees
    #mdiff.turn_to_angle(SpeedRPM(20), 90)
    # Disable odometry

#mdiff.odometry_stop()




print("Fin")



