#!/usr/bin/python3
from __future__ import print_function

from itertools import count
import rospy
from geometry_msgs.msg import Twist



from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
from arm_controller.srv import *


from arm_controller.srv import *
import rospy


'''
Spatial detection network demo.
    Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
'''


# if len(sys.argv) > 1:
#     nnBlobPath = sys.argv[1]

# if not Path(nnBlobPath).exists():
#     import sys
#     raise FileNotFoundError(
#         f'Required file/s not found, please run "{sys.executable} install_requirements.py"')




def inverseKin(cood):
    d1 = 45 #finger stepper motor offset
    d2 = 83 #vertical stepper motor offset
    d3 = 0 #dc motor offsets
    jointPara = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
    }
    # x = cood["x"]
    # y = cood["y"]
    # z = cood["z"]
    jointPara["x"] = (cood["x"] + d1)/1000
    jointPara["y"] = (cood["y"] + d2)/1000
    jointPara["z"] = (cood["z"] + d3)/1000

    return jointPara


# Pipeline is defined, now we can connect to the device


# MobilenetSSD label texts
labelMap = ["unknown",
        "button",
        "dashboard",
        "down",
        "G",
        "Nottingham logo",
        "one",
        "open",
        "pressed button"]

syncNN = True

# Get argument first
# nnBlobPath = str((Path(__file__).parent /
#                  Path('mobilenet-ssd_openvino_2021.2_6shave.blob')).resolve().absolute())

nnBlobPath = str((Path(__file__).parent /
                    Path('all5.blob')).resolve().absolute())

# Start defining a pipeline
pipeline = dai.Pipeline()

# Define a source - color camera
colorCam = pipeline.createColorCamera()
spatialDetectionNetwork = pipeline.createMobileNetSpatialDetectionNetwork()
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()

xoutRgb = pipeline.createXLinkOut()
xoutNN = pipeline.createXLinkOut()
xoutBoundingBoxDepthMapping = pipeline.createXLinkOut()
xoutDepth = pipeline.createXLinkOut()

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
xoutDepth.setStreamName("depth")

colorCam.setPreviewSize(300, 300)
colorCam.setResolution(
    dai.ColorCameraProperties.SensorResolution.THE_1080_P)
colorCam.setInterleaved(False)
colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(
    dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Setting node configs
stereo.setOutputDepth(True) 
stereo.setConfidenceThreshold(255)

# Better handling for occlusions:
stereo.setLeftRightCheck(True)
# Closer-in minimum depth, disparity range is doubled:
# stereo.setExtendedDisparity(False)

spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

# Create outputs

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

colorCam.preview.link(spatialDetectionNetwork.input)
if syncNN:
    spatialDetectionNetwork.passthrough.link(xoutRgb.input)
else:
    colorCam.preview.link(xoutRgb.input)

spatialDetectionNetwork.out.link(xoutNN.input)
spatialDetectionNetwork.boundingBoxMapping.link(
    xoutBoundingBoxDepthMapping.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

def callback(response):

    with dai.Device(pipeline) as device:
        # Start pipeline
        device.startPipeline()

        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        previewQueue = device.getOutputQueue(
            name="rgb", maxSize=4, blocking=False)
        detectionNNQueue = device.getOutputQueue(
            name="detections", maxSize=4, blocking=False)
        xoutBoundingBoxDepthMapping = device.getOutputQueue(
            name="boundingBoxDepthMapping", maxSize=4, blocking=False)
        depthQueue = device.getOutputQueue(
            name="depth", maxSize=4, blocking=False)

        frame = None
        detections = []

        startTime = time.monotonic()
        counter = 0
        counter2 = 0

        cood = {
            "count": 0,
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        }
        print(cood)
        fps = 0

        color = (255, 255, 255)

        while True:
            inPreview = previewQueue.get()
            inNN = detectionNNQueue.get()
            depth = depthQueue.get()

            counter += 1
            
            print(counter2)
            if counter2 == 80:
                break
            current_time = time.monotonic()
            if (current_time - startTime) > 1:
                fps = counter / (current_time - startTime)
                counter = 0
                startTime = current_time

            frame = inPreview.getCvFrame()
            depthFrame = depth.getFrame()

            depthFrameColor = cv2.normalize(
                depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            depthFrameColor = cv2.equalizeHist(depthFrameColor)
            depthFrameColor = cv2.applyColorMap(
                depthFrameColor, cv2.COLORMAP_HOT)
            detections = inNN.detections
            if len(detections) != 0:
                boundingBoxMapping = xoutBoundingBoxDepthMapping.get()
                roiDatas = boundingBoxMapping.getConfigData()

                for roiData in roiDatas:
                    roi = roiData.roi
                    roi = roi.denormalize(
                        depthFrameColor.shape[1], depthFrameColor.shape[0])
                    topLeft = roi.topLeft()
                    bottomRight = roi.bottomRight()
                    xmin = int(topLeft.x)
                    ymin = int(topLeft.y)
                    xmax = int(bottomRight.x)
                    ymax = int(bottomRight.y)

                    cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax,
                                                                  ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)

            # If the frame is available, draw bounding boxes on it and show the frame
            height = frame.shape[0]
            width = frame.shape[1]
           

            for detection in detections:
                if detection.label == 1:
                    counter2 += 1
                    if counter2 == 10:
                        cood["x"] = detection.spatialCoordinates.x
                        cood["y"] = detection.spatialCoordinates.y
                        cood["z"] = detection.spatialCoordinates.z
                    elif counter2 >= 11:
                        cood["count"] = counter2 - 10
                        cood["x"] = (
                            cood["x"] + detection.spatialCoordinates.x)/2
                        cood["y"] = (
                            cood["y"] + detection.spatialCoordinates.y)/2
                        cood["z"] = (
                            cood["z"] + detection.spatialCoordinates.z)/2

                    # Denormalize bounding box
                    x1 = int(detection.xmin * width)
                    x2 = int(detection.xmax * width)
                    y1 = int(detection.ymin * height)
                    y2 = int(detection.ymax * height)
                    try:
                        label = labelMap[detection.label]
                    except:
                        label = detection.label

                    cv2.putText(frame, str(label), (x1 + 10, y1 + 20),
                                cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                    cv2.putText(frame, "{:.2f}".format(detection.confidence*100),
                                (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                    cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm",
                                (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                    cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm",
                                (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                    cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm",
                                (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)

                    cv2.rectangle(frame, (x1, y1), (x2, y2),
                                color, cv2.FONT_HERSHEY_SIMPLEX)

            cv2.putText(frame, "NN fps: {:.2f}".format(
                fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
            # cv2.imshow("depth", depthFrameColor)
            # cv2.imshow("rgb", frame)
            


            if cv2.waitKey(20) == ord('q'):
                break
                
            # talker(cood["x"]/1000)


        # cv2.destroyWindow("rgb")
        print(cood)
        if cood["x"] == 0 and cood["y"] == 0 and cood["z"] == 0:
            respVision = VisionResponse(0, 0, False, True)
        else:
            joint = inverseKin(cood)
            respVision = VisionResponse(joint["x"], joint["y"], True, True)
    
    return respVision


def info():

    rospy.init_node('Visionserver')
    rospy.Service('Vision', Vision, callback)
    rospy.spin()


if __name__ == "__main__":
    info()
