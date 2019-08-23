#! /usr/bin/env python
import rospy
from lidar.msg import ControlCommand


###data read // GEAR, SPEED, STEER_G, STEER_C, BRAKE
GEAR=0
SPEED=0
STEER_G=0
STEER_C=0
BRAKE=0

def callback(data):
    global GEAR, SPEED, STEER_G, STEER_C, BRAKE
    GEAR = data.Gear
    SPEED = data.Speed
    STEER_G = data.Steer_G
    STEER_C = data.Steer_C
    BRAKE = data.Brake


if __name__ == '__main__': #edit
    
    rospy.init_node('Driving_node', anonymous=True)

    rospy.Subscriber("DRIVING_MODE", ControlCommand, callback)

    pub_controlcommand = rospy.Publisher('SERIAL_DATA', ControlCommand, queue_size=1)

    rospy.init_node('Driving_node', anonymous=True)
    
    rate = rospy.Rate(20)
    output = ControlCommand()
    #rospy.spin()

    while not rospy.is_shutdown():
        try:
            
            steer_final = STEER_G + STEER_C*0.05

            output.Gear=GEAR
            output.Speed=SPEED
            output.Steer_G=steer_final
            output.Steer_C=0
            output.Brake=BRAKE
            print (output.Gear, output.Speed, output.Steer_G)

            pub_controlcommand.publish(output)

        except rospy.ROSInterruptException:
            pass
