import rospy
import roslib

#import cv;
import cv2;
import cv_bridge

import numpy
import math
import os
import sys
import string
import time
import random
import tf
from sensor_msgs.msg import Image
import baxter_interface
from moveit_commander import conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest


''' GLOBALS '''
cv_image = None #cv2.createImage((self.width, self.height), 8, 3)


''' REGION FUNCTIONS '''

# reset all cameras (incase cameras fail to be recognised on boot)
def reset_cameras():
    reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
    rospy.wait_for_service('cameras/reset', timeout=10)
    reset_srv()

# open a camera and set camera parameters
def open_camera(camera, x_res, y_res):
    if camera == "left":
        cam = baxter_interface.camera.CameraController("left_hand_camera")
    elif camera == "right":
        cam = baxter_interface.camera.CameraController("right_hand_camera")
    elif camera == "head":
        cam = baxter_interface.camera.CameraController("head_camera")
    else:
        sys.exit("ERROR - open_camera - Invalid camera")

    # close camera
    # cam.close()

    # set camera parameters
    cam.resolution          = (int(x_res), int(y_res))
    cam.exposure            = -1             # range, 0-100 auto = -1
    cam.gain                = -1             # range, 0-79 auto = -1
    cam.white_balance_blue  = -1             # range 0-4095, auto = -1
    cam.white_balance_green = -1             # range 0-4095, auto = -1
    cam.white_balance_red   = -1             # range 0-4095, auto = -1

    # open camera
    cam.open()

# close a camera
def close_camera(camera):
    if camera == "left":
        cam = baxter_interface.camera.CameraController("left_hand_camera")
    elif camera == "right":
        cam = baxter_interface.camera.CameraController("right_hand_camera")
    elif camera == "head":
        cam = baxter_interface.camera.CameraController("head_camera")
    else:
        sys.exit("ERROR - close_camera - Invalid camera")

    # set camera parameters to automatic
    cam.exposure            = -1             # range, 0-100 auto = -1
    cam.gain                = -1             # range, 0-79 auto = -1
    cam.white_balance_blue  = -1             # range 0-4095, auto = -1
    cam.white_balance_green = -1             # range 0-4095, auto = -1
    cam.white_balance_red   = -1             # range 0-4095, auto = -1

    # close camera
    cam.close()

# camera call back function
def camera_callback(data, camera_name):
    global cv_image
    # Convert image from a ROS image message to a CV image
    try:
        cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8")
        cv2.imshow('Baxter', cv_image)
    except cv_bridge.CvBridgeError, e:
        print e

    # 3ms wait
    cv2.waitKey(3)

# left camera call back function
def left_camera_callback(data):
    camera_callback(data, "Left Hand Camera")

# right camera call back function
def right_camera_callback(data):
    camera_callback(data, "Right Hand Camera")

# head camera call back function
def head_camera_callback(data):
    camera_callback(data, "Head Camera")

# create subscriber to the required camera
def subscribe_to_camera(camera):
    if camera == "left":
        callback = left_camera_callback
        camera_str = "/cameras/left_hand_camera/image"
    elif camera == "right":
        callback = right_camera_callback
        camera_str = "/cameras/right_hand_camera/image"
    elif camera == "head":
        callback = head_camera_callback
        camera_str = "/cameras/head_camera/image"
    else:
        sys.exit("ERROR - subscribe_to_camera - Invalid camera")

    camera_sub = rospy.Subscriber(camera_str, Image, callback)

''' END REGION '''



limb = 'right'
width = 960
height = 600

# initialise ros node
rospy.init_node("connect_dots", anonymous = True)

# Enable the actuators
baxter_interface.RobotEnable().enable()

# directory used to save analysis images
image_directory = os.getenv("HOME") + "/Dots/"

# reset cameras
reset_cameras()

# close all cameras
#close_camera("left")
#close_camera("right")
#close_camera("head")

# open required camera
open_camera(limb, width, height)

# subscribe to required camera
subscribe_to_camera(limb)

while True:
    time.sleep(0.1)
    #if cv_image is not None:
     #   print cv_image
        

