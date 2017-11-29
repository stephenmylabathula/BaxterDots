# Python Imports
import sys
import copy
import time
import struct
import argparse

# ROS Imports
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, Empty

# Baxter Imports
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import baxter_interface



''' CLASS DEFINITION : BaxterIK '''

class BaxterIK(object):
    def __init__(self, limb, verbose=True):
        self._limb_name = limb
        self._verbose = verbose
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def solve_ik(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)
 
    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

''' END CLASS DEFINITION '''



''' MAIN CODE STARTS HERE '''

rospy.init_node("baxter_ik")	# create ROS node

# string definitions for each arm
left_arm = 'left'
right_arm = 'right'

# create a baxter IK object
bik = BaxterIK(right_arm)	# replace with left arm if you want to plan for that
#time.sleep(1)
#bik.gripper_close()
#time.sleep(1)
# Define a Pose to Plan to
goal_pose = Pose(position=Point(x=0.55, y=0.2, z=-0.195), orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0))

# Generate joint variables for target pose
goal_joints = bik.solve_ik(goal_pose)

# Plan a path and move to joint positions
bik.guarded_move_to_joint_position(goal_joints)

''' CODE ENDS HERE '''





