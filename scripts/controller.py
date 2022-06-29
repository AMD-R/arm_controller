#!/usr/bin/env python3
"""
Stepper Message - Int16MultiArray
ind 0 - Command 
    1 - RPM
    2 - Steps
    
Commands
0 - Move
1 - Home

Linear Message - Bool
Commands
0 - Retract
1 - Extend

Linear Stop Message - Empty
"""

import rospy
import math
from std_msgs.msg import Int16MultiArray, Bool, Empty
from arm_controller.srv import *

NAMESPACE = "/arm"

# PUB TOPICS
TOPIC_MOTOR_X = NAMESPACE + "/motor_x"
TOPIC_COMMAND_X = TOPIC_MOTOR_X + "/motor_cmd"
TOPIC_STOP_X = TOPIC_MOTOR_X + "/motor_stop"
TOPIC_MOTOR_STEPPER = NAMESPACE + "/stepper"
TOPIC_STEPPER_COMMAND = TOPIC_MOTOR_STEPPER + "/motor_cmd"

# SERVICE TOPIC
SERVICE_CMD = NAMESPACE + "/arm_cmd"
SERVICE_STOP = NAMESPACE + "/x_stop"

# PROTOCALL
COMMAND_MOVE = 0
COMMAND_HOME = 1
INDEX_CMD = 0
INDEX_RPM_Y = 1
INDEX_STEPS_Y = 2
INDEX_RPM_Z = 3
INDEX_STEPS_Z = 4

# MOTOR CONSTANTS
M_TO_STEPS_Z = 100000
M_TO_STEPS_Y = 40000


def handle_move_arm(req):
    motor_cmd_stepper = Int16MultiArray()
    motor_cmd_X = Bool()
    
    motor_cmd_stepper.data = [0,0,0,0,0]

    motor_cmd_stepper.data[INDEX_CMD] = COMMAND_MOVE

    motor_cmd_stepper.data[INDEX_RPM_Y] = req.rpm_y
    motor_cmd_stepper.data[INDEX_RPM_Z] = req.rpm_z

    motor_cmd_stepper.data[INDEX_STEPS_Z] = round(req.dist_z * M_TO_STEPS_Z)
    motor_cmd_stepper.data[INDEX_STEPS_Y] = round(req.dist_y * M_TO_STEPS_Y)

    motor_cmd_X.data = req.cmd_x

    pub_stepper.publish(motor_cmd_stepper)
    #pub_Z.publish(motor_cmd_Z)
    #pub_Y.publish(motor_cmd_Y)
    pub_move_X.publish(motor_cmd_X)
    
    print("Moving Arm by [y:%s z:%s]" % (req.dist_y, req.dist_z))

    resp = ArmResponse(True, 1)

    return resp

    # return !XXXXXXXXXXXXXXXXXSRV FILE RESPONSE XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX (req.a + req.b)
    # TODO: Write/import
    

def handle_stop_motor_x(req):
    msg = Empty()
    print("Stop")
    pub_stop_X.publish(msg)


def arm_controller():
    global pub_stepper, pub_move_X, pub_stop_X
    #global pub_Z, pub_Y, pub_move_X, pub_stop_X
    # pub_Z = rospy.Publisher(TOPIC_COMMAND_Z, Int16MultiArray, queue_size=10)
    pub_stepper = rospy.Publisher(TOPIC_STEPPER_COMMAND, Int16MultiArray, queue_size=10)
    pub_move_X = rospy.Publisher(TOPIC_COMMAND_X, Bool, queue_size=10)
    pub_stop_X = rospy.Publisher(TOPIC_STOP_X, Empty, queue_size=10)

    rospy.init_node('arm_controller', anonymous=True)
    # TODO: Write/import
    rospy.Service(SERVICE_CMD, Arm, handle_move_arm)
    # TODO: Write/import
    rospy.Service(SERVICE_STOP, Stop, handle_stop_motor_x)
    rate = rospy.Rate(50)
    rospy.spin()


if __name__ == '__main__':
    try:
        arm_controller()
    except rospy.ROSInterruptException:
        pass