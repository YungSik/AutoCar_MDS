#! /usr/bin/env python

import rospy
from lidar.msg import ControlCommand
from math import *

import math
import time
import serial


#ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
#                    bytesize=serial.EIGHTBITS, timeout=1)

S = chr(0x53)
T = chr(0x54)
X = chr(0x58)
AorM = chr(0x01)
ESTOP = chr(0x00)
GEAR = chr(0x00)
SPEED0 = chr(0x00)
SPEED1 = chr(0x00)
STEER0 = chr(0X02)
STEER1 = chr(0x02)
BRAKE = chr(0x01)
ALIVE = 0
ETX0 = chr(0x0d)
ETX1 = chr(0x0a)
Packet=[]
read=[]
count=0
count_alive=0
##################################################
def GetAorM():
    AorM = chr(0x01)
    return  AorM

def GetESTOP():
    ESTOP = chr(0x00)
    return  ESTOP

def GetGEAR(gear):
    GEAR = chr(gear)
    return  GEAR

def GetSPEED(speed):
    global count
    SPEED0 = chr(0x00)
    SPEED1 = chr(speed)
    return SPEED0, SPEED1

def GetSTEER(steer):
    steer=steer*71
    steer_max=0b0000011111010000#+2000
    steer_0 = 0b0000000000000000
    steer_min=0b1111100000110000#-2000

    #print(steer)
    if (steer>=0):
        angle=int(steer)
        STEER=steer_0+angle
    else:
        angle=int(-steer)
        angle=2000-angle
        STEER=steer_min+angle

    STEER0=STEER & 0b1111111100000000
    STEER0=STEER0 >> 8
    STEER1=STEER & 0b0000000011111111

    return chr(STEER0), chr(STEER1)

def GetBRAKE(brake):
    BRAKE = chr(brake)
    return  BRAKE

def Send_to_ERP42(Gear, Speed, Steer, Brake):
    global S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1, count_alive

    count_alive = count_alive+1
    if count_alive==0xff:
        count_alive=0x00
    print("{}, {}".format(steer,speed))

    AorM = GetAorM()
    ESTOP = GetESTOP()
    GEAR = GetGEAR(Gear)
    SPEED0, SPEED1 = GetSPEED(Speed)
    STEER0, STEER1 = GetSTEER(Steer)
    BRAKE = GetBRAKE(Brake)

    ALIVE = chr(count_alive)
    vals = [S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1]

    for i in range(len(vals)):
        ser.write(vals[i])
    #time.sleep(0.2)
def Read_from_ERP42():
    global Packet

    Packet=ser.readline()


################################################################################
gear = 0
speed = 0
steer = 0
brake = 0

def callback(data):
    global gear, speed, steer, brake
    gear = data.Gear
    speed = data.Speed
    steer = data.Steer_G
    trash = data.Steer_C
    brake = data.Brake
    print(gear,speed,steer,brake)

def listener():
    rospy.init_node('Serial_node', anonymous=True)

    rospy.Subscriber("SERIAL_DATA", ControlCommand, callback)

    #rospy.spin()

if __name__ == '__main__':
    listener()
    port = rospy.get_param("~CTRL_PORT",default="/dev/ttyUSB0")
    print(port)
    ser = serial.serial_for_url(port, baudrate=115200, timeout=1)

    while (ser.isOpen() and (not rospy.is_shutdown())):
        Send_to_ERP42(gear, speed, steer, brake)
        Read_from_ERP42()