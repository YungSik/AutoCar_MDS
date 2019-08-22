#! /usr/bin/env python
import rospy
from lidar.msg import ControlCommand


###data read // GEAR, SPEED, STEER_G, STEER_C, BRAKE
GEAR=0
STEER=0
SPEED_G=0
STEER_C=0
BRAKE=0

def callback(data):
    global GEAR, SPEED, STEER_G, STEER_C, BRAKE
    GEAR = data.Gear
    SPEED = data.Speed
    STEER_G = data.Steer_G
    STEER_C = data.Steer_C
    BRAKE = data.Brake


def listener():
    rospy.init_node('Driving_node', anonymous=True)

    rospy.Subscriber("DRIVING_MODE", ControlCommand, callback)

    rospy.spin()

def talker():
    pub_controlcommand = rospy.Publisher('driving_mode', ControlCommand, queue_size=1)

    rospy.init_node('position', anonymous=True)
    rate = rospy.Rate(20)


if __name__ == '__main__': #edit
    
    listener()
    talker()

    while not rospy.is_shutdown():
        try:
            steer_final = STEER_G + STEER_C*0.05

            driving_mode.GEAR=GEAR
            driving_mode.SPEED=SPEED
            driving_mode.STEER_G=steer_final
            driving_mode.STEER_C=0
            driving_mode.BRAKE=BRAKE

            pub_controlcommand.publish(driving_mode)

        except rospy.ROSInterruptException:
            pass