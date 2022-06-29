# #!/usr/bin/env python3


# from __future__ import print_function


# from arm_controller.srv import *
# import rospy
# from std_msgs.msg import Empty


# def callback(response):
#     # button coordinate
#     # global x, y, z, isActive, resultVision
#     # print(response)
#     # x = response.x
#     # y = response.y
#     # z = response.z
#     # isActive = response.isActive
#     print(response)
#     resp = ArmResponse(True, 1)

#     return resp


# def callbackStop(response):

#     print(Empty)
#     # resp = StopResponse()
#     # print(response)
#     # print(type(response))
#     # return resp


# def info():

#     rospy.init_node('Armserver')
#     rospy.Service('arm/arm_cmd', Arm, callback)
#     rospy.Service('arm/x_stop', Stop, callbackStop)

#     rospy.spin()


# if __name__ == "__main__":
#     info()
