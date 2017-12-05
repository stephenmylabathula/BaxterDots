# ROS Imports
import tf
import rospy
import roslib
import std_srvs.srv
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# Baxter Imports
import baxter_interface
from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

# OpenCV Imports
import cv2
import cv_bridge

# Python Imports
import os
import sys
import time
import math
import string
import random
import numpy as np


# TODO: Add staticmethod directive to static functions
# TODO: Fix pixel_to_baxter function
# TODO: Move out right arm if overhead  initially


class RobotMotion:

    """
    Class that provides a motion control interface for Baxter.
    This class is tailored for Baxter in Keller UG Robotics Lab
    """

    def __init__(self):

        # Init ROS Node
        rospy.init_node("RobotMotion", anonymous=True)

        # Setup Limbs
        self.left_limb = 'left'
        self.left_limb_interface = baxter_interface.Limb(self.left_limb)
        self.right_limb = 'right'
        self.right_limb_interface = baxter_interface.Limb(self.right_limb)

        # Setup Right Hand Gripper
        self.gripper = baxter_interface.Gripper(self.right_limb)

        # IR Distance
        self.distance = 0.0

        # Enable Actuators
        baxter_interface.RobotEnable().enable()

        # Set Speed Percentage
        self.left_limb_interface.set_joint_position_speed(0.5)
        self.right_limb_interface.set_joint_position_speed(0.5)

    # Move Limb
    def ik_move(self, limb, rpy_pose, print_result=False):
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(quaternion_pose)

        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            print "ERROR - baxter_ik_move - Failed to append pose"
        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if limb == 'left':
                self.left_limb_interface.move_to_joint_positions(limb_joints)
            elif limb == 'right':
                self.right_limb_interface.move_to_joint_positions(limb_joints)
        else:
            print "ERROR - baxter_ik_move - No valid joint configuration found"

        if print_result:
            # Report Move Accuracy
            if limb == 'left':
                quaternion_pose = self.left_limb_interface.endpoint_pose()
            elif limb == 'right':
                quaternion_pose = self.right_limb_interface.endpoint_pose()

            position = quaternion_pose['position']
            quaternion = quaternion_pose['orientation']
            euler = tf.transformations.euler_from_quaternion(quaternion)

            print "             request   actual"
            print 'front back = %5.4f ' % rpy_pose[0], "%5.4f" % position[0]
            print 'left right = %5.4f ' % rpy_pose[1], "%5.4f" % position[1]
            print 'up down    = %5.4f ' % rpy_pose[2], "%5.4f" % position[2]
            print 'roll       = %5.4f ' % rpy_pose[3], "%5.4f" % euler[0]
            print 'pitch      = %5.4f ' % rpy_pose[4], "%5.4f" % euler[1]
            print 'yaw        = %5.4f ' % rpy_pose[5], "%5.4f" % euler[2]

    # Get IR Distance
    def get_distance(self, limb):
        if limb == "left":
            dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
        elif limb == "right":
            dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
        else:
            sys.exit("ERROR - get_distance - Invalid limb")

        if dist > 65000:
            dist = 500
            print "ERROR - get_distance - no distance found"

        return float(dist / 1000.0)

    # Calibrate Right Hand Gripper
    def calibrate_gripper(self):
        self.gripper.calibrate()

    # Close Gripper
    def close_gripper(self):
        self.gripper.close()

    # Open Gripper
    def open_gripper(self):
        self.gripper.open()


class RobotVision:

    """
    Class that provides interface into Baxter arm cameras.
    This class is tailored for the connect the dots project.
    """

    def __init__(self):

        # Init ROS Node (Uneccesary?)
        # rospy.init_node("RobotVision", anonymous=True)

        # Subscription Handle to Camera
        self.camera_sub = None

        self.num_dots_found = 0     # Current Number of Dots Found in Image
        self.current_dots = None    # Current Array of Dots Found

        self.reset_cameras()

    def reset_cameras(self):
        reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
        rospy.wait_for_service('cameras/reset', timeout=10)
        reset_srv()

    def open_camera(self, camera, x_res, y_res):
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        else:
            sys.exit("ERROR - open_camera - Invalid camera")

        # set camera parameters
        cam.resolution = (int(x_res), int(y_res))
        cam.exposure = -1  # range, 0-100 auto = -1
        cam.gain = -1  # range, 0-79 auto = -1
        cam.white_balance_blue = -1  # range 0-4095, auto = -1
        cam.white_balance_green = -1  # range 0-4095, auto = -1
        cam.white_balance_red = -1  # range 0-4095, auto = -1

        # open camera
        cam.open()

    def camera_callback(self, data, camera_name):
        """ Callback function for when image data is ready."""
        try:
            # convert image from a ROS image message to a CV image
            cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8")

            # process image through blob detector
            proc_image, points = self.blob_detector(cv_image)

            # update globals
            self.num_dots_found = len(points)
            self.current_dots = points

            # show processed image
            cv2.imshow('Baxter', proc_image)
        except cv_bridge.CvBridgeError, e:
            print e
        cv2.waitKey(3)      # 3ms Wait

    def left_camera_callback(self, data):
        self.camera_callback(data, "Left Hand Camera")

    def right_camera_callback(self, data):
        self.camera_callback(data, "Right Hand Camera")

    def subscribe_to_camera(self, camera):
        """ Subscribes to a given camera feed. """
        if camera == "left":
            callback = self.left_camera_callback
            camera_str = "/cameras/left_hand_camera/image"
        elif camera == "right":
            callback = self.right_camera_callback
            camera_str = "/cameras/right_hand_camera/image"
        else:
            sys.exit("ERROR - subscribe_to_camera - Invalid camera")

        self.camera_sub = rospy.Subscriber(camera_str, Image, callback)

    def blob_detector(self, img):
        """ Returns Image with Blobs Outlined

            pts - is an array of tuples of the form (x,y)
                  which is the center of the blob of type float
        """

        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # params = cv2.SimpleBlobDetector_Params()    # can pass into SimpleBlobDetector_create()

        detector = cv2.SimpleBlobDetector_create()
        keypoints = detector.detect(gray_img)
        pts = [i.pt for i in keypoints]
        img = cv2.drawKeypoints(img, keypoints, np.array([]), (0, 0, 255),
                                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return img, pts

    def contour_detector(self, img):
        """ Gets Contours of Image with Blue Threshold Out """

        # Threshold and Blur/Erode
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv = cv2.inRange(hsv, (90, 100, 0), (135, 255, 255))
        hsv = cv2.medianBlur(hsv, 3)
        hsv = cv2.erode(hsv, np.ones((3, 3), np.uint8), iterations=1)
        hsv = cv2.medianBlur(hsv, 7)

        # Find/Draw Contours
        im2, contours, hierarchy = cv2.findContours(hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (0, 255, 0), 3)

        return img

    def hough_transform(self, img):
        """ Finds and Draws Hough Circles in Image """

        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_img = cv2.medianBlur(gray_img, 5)

        circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=110, minRadius=0, maxRadius=0)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)

        return img

    def find_dots(self, num_dots):

        """ Finds the Coordinates of Dots in Image """

        self.open_camera('left', 960, 600)
        self.subscribe_to_camera('left')

        # Keep running until we find the correct number of dots
        while self.num_dots_found != num_dots:
            time.sleep(0.1)

        # Cleanup
        cv2.destroyAllWindows()
        self.camera_sub.unregister()    # Unsubscribe from Camera
        return self.current_dots


# convert image pixel to Baxter point
def pixel_to_baxter(px, dist, cam_calib=0.0025, cam_x_offset=0.045, cam_y_offset=-0.01):
    x = ((px[1] - (600 / 2)) * cam_calib * dist) + 0.6 + cam_x_offset
    y = ((px[0] - (960 / 2)) * cam_calib * dist) + 0.0 + cam_y_offset
    return x, y


def main():

    overhead_pose = [0.6, 0.0, 0.1, math.pi, 0.0, 0.0]      # Change Yaw for Wrist Rotation

    # Move Out Right Arm
    print "Moving Right Arm to Side"
    motion = RobotMotion()
    pose = [0.2, -1.0, 0.2, -math.pi, 0.0, 0.0]  # Right Arm to Side Pose
    motion.calibrate_gripper()
    motion.ik_move('right', pose)
    time.sleep(2)
    motion.close_gripper()

    # Move Left Arm to Start Position
    print "Moving Left Arm to Start Position"
    pose = overhead_pose     # Overhead Pose
    motion.ik_move('left', pose)
    time.sleep(1)
    camera_table_distance = motion.get_distance('left')
    print camera_table_distance

    # Start Camera
    print "Opening Left Camera"
    vision = RobotVision()
    dot_locations = vision.find_dots(6)     # Find 6 Dots
    print dot_locations
    dot_locations = np.array(dot_locations, dtype=np.int16)
    print "Found All Dots!"

    # Move Out Left Arm
    print "Moving Left Arm to Side"
    pose = [0.2, 1.0, 0.2, -math.pi, 0.0, 0.0]      # Left Arm to Side Pose
    motion.ik_move('left', pose)
    time.sleep(1)

    # Move In Right Arm to Start Connecting Dots
    print "Moving Right Arm In"
    pose = overhead_pose     # Overhead Pose
    motion.ik_move('right', pose)
    time.sleep(1)

    # Map Pixel to Baxter
    baxter_points = np.tile([0.0, 0.0, -0.15, -math.pi, 0.0, 0.0], (6, 1))
    for i in range(len(dot_locations)):
        baxter_points[i][:2] = pixel_to_baxter(dot_locations[i], camera_table_distance)
    print baxter_points

    # Move Right Arm to Dot Locations
    for i in baxter_points:
        motion.ik_move('right', i)
        time.sleep(1)

    # Move Out Right Arm
    print "Moving Right Arm to Side"
    motion.open_gripper()
    pose = [0.2, -1.0, 0.2, -math.pi, 0.0, 0.0]  # Right Arm to Side Pose
    motion.ik_move('right', pose)
    time.sleep(1)



if __name__ == "__main__":
    main()
