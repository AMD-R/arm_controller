#!/usr/bin/env python3
import rospy
import time
from arm_controller.srv import *
from std_msgs.msg import Float32, Int16

NAMESPACE = "/arm"
# SERVICE TOPIC
SERVICE_CMD = NAMESPACE + "/arm_cmd"
SERVICE_STOP = NAMESPACE + "/x_stop"

def clientVision(resultVision: bool):
    """Service proxy for Vision (OAK-D-LITE)."""
    rate = rospy.Rate(1)
    try:        
        global visionResp

        rospy.wait_for_service('Vision')

        service: Vision = rospy.ServiceProxy(
            'Vision', Vision)
        visionResp = service(resultVision)

        rate.sleep()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def clientArm(dist_x: Float32, dist_y: Float32, dist_z: Float32,
              rpm_y: Int16, rpm_z: Int16, cmd_x: bool):
    """Service proxy for /arm/arm_cmd (robotic arm)."""
    rate = rospy.Rate(1)
    global armResp
    try:
        rospy.wait_for_service(SERVICE_CMD)

        service: Arm = rospy.ServiceProxy(
            SERVICE_CMD, Arm)
        armResp = service(dist_x, dist_y, dist_z, rpm_y, rpm_z, cmd_x)
        rate.sleep()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def clientNav(resultNav: bool):
    """Service proxy for Nav (Navigation)."""
    rate = rospy.Rate(1)
    global navResp
    try:
        rospy.wait_for_service('Nav')

        service: Nav = rospy.ServiceProxy(
            'Nav', Nav)
        navResp = service(resultNav)
        rate.sleep()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def clientHMI(resultHMI: bool):
    """Service proxy for HMI."""
    rate = rospy.Rate(1)
    global hmiResp
    try:
        rospy.wait_for_service('HMI')

        service: HMI = rospy.ServiceProxy(
            'HMI', HMI)
        hmiResp = service(resultHMI)
        rate.sleep()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def clientStop():
    """Service proxy for /arm/x_stop (stopping)."""
    rate = rospy.Rate(1)
    global stopResp
    try:
        rospy.wait_for_service(SERVICE_STOP)

        service: Stop = rospy.ServiceProxy(
            SERVICE_STOP, Stop)
        stopResp = service()
        rate.sleep()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def clientButton():
    """Service proxy for buttonStatus."""
    rate = rospy.Rate(1)
    global buttonResp
    try:
        rospy.wait_for_service('buttonStatus')

        service: buttonStatus = rospy.ServiceProxy(
            'buttonStatus', buttonStatus)
        buttonResp = service(True)
        rate.sleep()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)        


if __name__ == "__main__":
    rospy.init_node("Server")
    global visionResp, buttonResp, stopResp, hmiResp, navResp, armResp

    clientNav(True)
    clientVision(True)
    if visionResp.z == True:
        clientArm(0, visionResp.x, visionResp.y , 300, 200, False)
        timeY = visionResp.x*420
        timeZ = visionResp.y*160
        if timeY >= timeZ:
            time.sleep(timeY)
        elif timeZ >= timeY:
            time.sleep(timeZ)
        clientArm(0, 0, 0 , 300, 200, True)
        time.sleep(15)
        clientArm(0, 0, 0 , 300, 200, False)
        time.sleep(2)
        clientArm(0, -visionResp.x, -visionResp.y, 300, 200, False)
   
    clientArm(0,0.05,0,300,200,False)

    # clientVision(True)
    # if visionResp.z == True:
    #     clientArm(0, visionResp.x, visionResp.y , 300, 120, False)
    #     time.sleep(26)
    #     clientArm(0, 0, 0 , 300, 120, True)
