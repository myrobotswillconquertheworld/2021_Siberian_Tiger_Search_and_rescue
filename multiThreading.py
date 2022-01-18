#!/usr/bin/env python3

from time import sleep
import threading
import math
import ctypes
from smbus import SMBus
from ev3dev2.sound import Sound
from ev3dev2.sensor import INPUT_1, INPUT_4, INPUT_3
from ev3dev2.sensor.lego import TouchSensor, UltrasonicSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveDifferential, MoveTank, SpeedRPM
from ev3dev2.port import LegoPort
from ev3dev2.wheel import EV3Tire

#-------CAMERA----------
# Init camera on Input
in1 = LegoPort(INPUT_1)
in1.mode = 'other-i2c'
# Short wait for port to get ready
sleep(0.5)
# Settings for I2C (SMBus(3) for INPUT_1)
bus = SMBus(3)
# address set in Pixy2
address = 0x54
# Signatures we're interested in
sig1 = 1
sig2 = 2
sig3 = 3

#-----------TOUCH SENSOR/SPEAKER/MOTORS---------
ts = TouchSensor(INPUT_4)
spkr = Sound()
spkr.set_volume(100)
rmotor = LargeMotor(OUTPUT_B)
lmotor = LargeMotor(OUTPUT_A)
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
grab = LargeMotor(OUTPUT_C)
calibrateRotation = 88 #100 avec roues bien propre sur parquet (ajuster si besoin)
speed = SpeedRPM(40)
mdiff = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, calibrateRotation)

#----------OBJECTS DETECTIONS-------------
X_REF = 158  # X-coordinate of referencepoint
Y_REF = 150  # Y-coordinate of referencepoint
KP = 0.1     # Proportional constant PID-controller
KI = 0.01    # Integral constant PID-controller
KD = 0.05    # Derivative constant PID-controller
GAIN = 5    # Gain for motorspeed
integral_x = 0
derivative_x = 0
last_dx = 0
integral_y = 0
derivative_y = 0
last_dy = 0
data1 = [174, 193, 32, 2, sig1, 1] # Data for requesting block sig1,sig2 and sig3
data2 = [174, 193, 32, 2, sig2, 1]
data3 = [174, 193, 32, 2, sig3, 1]

#------------CONSTANTS FOR PATTERNS------------
coeff = 10.5
sideLength = 100 #side length of the area
sideOdo = coeff * sideLength
sidePart = sideOdo/4
#parcours 1/2
#X = [0,2*sidePart,2*sidePart,sideOdo,sideOdo]
#Y = [sideOdo,sideOdo,0,0,sideOdo]
#parcours 1/3
#X = [0        ,sideOdo/3  ,sideOdo/3  ,2*(sideOdo/3)  ,2*(sideOdo/3)  ,sideOdo  ,sideOdo]
#Y = [sideOdo  ,sideOdo    ,0          ,0              ,sideOdo        ,sideOdo  ,0]
#parcours 1/4
X = [0        ,sidePart  ,sidePart  ,2*sidePart  ,2*sidePart  ,3*sidePart  ,3*sidePart  ,sideOdo  ,sideOdo]
Y = [sideOdo  ,sideOdo   ,0         ,0           ,sideOdo     ,sideOdo     ,0           ,0        ,sideOdo] 

#--------------------------DETECTIONS FUNCTIONS----------------------------------
def checkBlueBall():
    global find
    global blueBallOnField

    while find is False:
        # Request block
        bus.write_i2c_block_data(address, 0, data2)
        # Read block
        block = bus.read_i2c_block_data(address, 0, 20)
        if sig2 == block[7]*256 + block[6]:
            print("I see a Blue Ball")
            blueBallOnField = True  

def checkGoldBall():
    global find
    global goldBallOnField

    while find is False:
        # Request block
        bus.write_i2c_block_data(address, 0, data3)
        # Read block
        block = bus.read_i2c_block_data(address, 0, 20)
        if sig3 == block[7]*256 + block[6]:
            print("I see a Gold Ball")
            goldBallOnField = True

def saveRedBall(integral_x,derivative_x,last_dx,integral_y,derivative_y,last_dy,index):
    global find
    detectTime = 0
    nbrRotation = 0
    arrive = 0

    while not ts.value():
        # Request block
        bus.write_i2c_block_data(address, 0, data1)
        # Read block
        block = bus.read_i2c_block_data(address, 0, 20)
        if sig1 == block[7]*256 + block[6]:
            print("detect")
            find = True
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
                    bus.write_i2c_block_data(address, 0, data1)
                    # Read block
                    block = bus.read_i2c_block_data(address, 0, 20)
                    if sig1 == block[7]*256 + block[6]:
                        print('je te vois toujours')
                        tank_drive.on_for_rotations(SpeedPercent(20),SpeedPercent(20), 1)
                        nbrRotation = nbrRotation + 1
                    else:
                        grab.on_for_rotations(SpeedRPM(30), 0.15)
                        print("je rentre a la maison")
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
                            mdiff.turn_to_angle(speed, 90)
                        elif index == 2 or index == 4 or index == 6 or index == 8:
                            mdiff.turn_to_angle(speed, 0)
                        elif index == 3 or index == 7 : 
                            mdiff.turn_to_angle(speed, 270)
                        return mdiff.on_to_coordinates(speed, 0, 0)

def saveBlueBall(integral_x,derivative_x,last_dx,integral_y,derivative_y,last_dy,index):
    global find
    detectTime = 0
    nbrRotation = 0
    arrive = 0

    while not ts.value():
        # Request block
        bus.write_i2c_block_data(address, 0, data2)
        # Read block
        block = bus.read_i2c_block_data(address, 0, 20)
        if sig2 == block[7]*256 + block[6]:
            print("detect")
            find = True
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
                    bus.write_i2c_block_data(address, 0, data2)
                    # Read block
                    block = bus.read_i2c_block_data(address, 0, 20)
                    if sig2 == block[7]*256 + block[6]:
                        print('je te vois toujours')
                        tank_drive.on_for_rotations(SpeedPercent(20),SpeedPercent(20), 1)
                        nbrRotation = nbrRotation + 1
                    else:
                        grab.on_for_rotations(SpeedRPM(30), 0.15)
                        print("je rentre a la maison")
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
                            mdiff.turn_to_angle(speed, 90)
                        elif index == 2 or index == 4 or index == 6 or index == 8:
                            mdiff.turn_to_angle(speed, 0)
                        elif index == 3 or index == 7 : 
                            mdiff.turn_to_angle(speed, 270)
                        return mdiff.on_to_coordinates(speed, 0, 0)

def saveGoldBall(integral_x,derivative_x,last_dx,integral_y,derivative_y,last_dy,index):
    global find
    detectTime = 0
    nbrRotation = 0
    arrive = 0

    while not ts.value():
        # Request block
        bus.write_i2c_block_data(address, 0, data3)
        # Read block
        block = bus.read_i2c_block_data(address, 0, 20)
        if sig3 == block[7]*256 + block[6]:
            print("detect")
            find = True
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
                    bus.write_i2c_block_data(address, 0, data3)
                    # Read block
                    block = bus.read_i2c_block_data(address, 0, 20)
                    if sig3 == block[7]*256 + block[6]:
                        print('je te vois toujours')
                        tank_drive.on_for_rotations(SpeedPercent(20),SpeedPercent(20), 1)
                        nbrRotation = nbrRotation + 1
                    else:
                        grab.on_for_rotations(SpeedRPM(30), 0.15)
                        print("je rentre a la maison")
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
                            mdiff.turn_to_angle(speed, 90)
                        elif index == 2 or index == 4 or index == 6 or index == 8:
                            mdiff.turn_to_angle(speed, 0)
                        elif index == 3 or index == 7 : 
                            mdiff.turn_to_angle(speed, 270)
                        return mdiff.on_to_coordinates(speed, 0, 0)


#--------------------------PATTERNS FUNCTIONS----------------------------------
def square():
    mdiff.on_to_coordinates(speed, 0, sideOdo/3)
    mdiff.on_to_coordinates(speed, sideOdo/3, sideOdo/3)
    mdiff.on_to_coordinates(speed, sideOdo/3, 0)
    mdiff.on_to_coordinates(speed, 0, 0)
    mdiff.turn_to_angle(speed, 90)

def pattern1():
    global index
    for i in range(len(X)):
        index = i
        mdiff.on_to_coordinates(speed, X[i], Y[i])
   
def arc():
    mdiff.turn_to_angle(SpeedRPM(5), 90)

def research():
    arc()
    pattern1()


#--------------------------OTHERS FUNCTIONS----------------------------------
def limit_speed(speed):
    """ Limit speed in range [-1000,1000] """
    if speed > 1000:
        speed = 1000
    elif speed < -1000:
        speed = -1000
    return speed

def kill_thread(thread):
    """
    thread: a threading.Thread object
    """
    thread_id = thread.ident
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, ctypes.py_object(SystemExit))
    if res > 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0)
        print('Exception raise failure')


#-------STARTING EXECUTION----------------------------------
print('robot is ready')
spkr.speak('Lets go')

mdiff.odometry_start(theta_degrees_start=0.0, x_pos_start=0.0, y_pos_start=0.0, sleep_time=0.005)
index = 0
nbrBall = 0
find = False
blueBallOnField = False
goldBallOnField = False

# creating thread
search1 = threading.Thread(target=research)
search2 = threading.Thread(target=research)
search3 = threading.Thread(target=research)

checkBlue = threading.Thread(target=checkBlueBall)
checkGold = threading.Thread(target=checkGoldBall)

saveRed = threading.Thread(target=saveRedBall, args=(0,0,0,0,0,0, index))
saveBlue = threading.Thread(target=saveBlueBall, args=(0,0,0,0,0,0, index))
saveGold = threading.Thread(target=saveGoldBall, args=(0,0,0,0,0,0, index))

search1.start()
saveRed.start()
checkBlue.start()
checkGold.start()

while find != True: #If the robot found the red ball, I stop the research
    a = 0

kill_thread(search1)
kill_thread(checkBlue)
kill_thread(checkGold)

# wait until he rescue the red ball
saveRed.join()

mdiff.turn_to_angle(speed, 0)

print("bleu : " + str(blueBallOnField))
print("gold : " + str(goldBallOnField))

if (blueBallOnField is True and goldBallOnField is True) or nbrBall == 3:
    print("il y a deux balles")
    #go rescue blue ball
    grab.on_for_rotations(SpeedRPM(30), -0.15)
    kill_thread(saveRed)
    index = 0
    find = False
    search2.start()
    saveBlue.start()
    while find != True:
        a = 0
    kill_thread(search2)
    saveBlue.join()
    #go rescue gold ball
    grab.on_for_rotations(SpeedRPM(30), -0.15)
    kill_thread(saveBlue)
    index = 0
    find = False
    search3.start()
    saveGold.start()
    while find != True:
        a = 0
    kill_thread(search3)
    saveGold.join()
elif blueBallOnField is True and nbrBall == 2:
    print("il y a seulement une balle bleu")
    #go rescue blue ball
    grab.on_for_rotations(SpeedRPM(30), -0.15)
    kill_thread(saveRed)
    index = 0
    find = False
    search2.start()
    saveBlue.start()
    while find != True:
        a = 0
    kill_thread(search2)
    saveBlue.join()
elif goldBallOnField is True and nbrBall == 2:
    print("il y a seulement une balle doree")
    #go rescue gold ball
    grab.on_for_rotations(SpeedRPM(30), -0.15)
    kill_thread(saveBlue)
    index = 0
    find = False
    search3.start()
    saveGold.start()
    while find != True:
        a = 0
    kill_thread(search3)
    saveGold.join()

mdiff.turn_to_angle(speed, 0)
rmotor.stop()
lmotor.stop()
print("fin")

