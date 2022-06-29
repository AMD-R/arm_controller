#!/usr/bin/env python3


from __future__ import print_function


from arm_controller.srv import *
import rospy


def callback(response):
    # button coordinate
    # global x, y, z, isActive, resultVision
    # print(response)
    # x = response.x
    # y = response.y
    # z = response.z
    # isActive = response.isActive
    print(response)
    resp = HMIResponse(True, 1)

    return resp


def info():

    rospy.init_node('HMIserver')
    rospy.Service('HMI', HMI, callback)

    rospy.spin()


if __name__ == "__main__":
    info()
