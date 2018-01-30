#!/usr/bin/env python

import rospy
# import pi_tracker_lib as PTL
from skeleton_markers.msg import Skeleton
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from robotis_controller_msgs import GetJointModule
# from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
# from pi_tracker.srv import *
from math import acos, asin, pi
import PyKDL as KDL
# import op3_tracker_lib as OP3TL

from robotis_controller_msgs import GetJointModule

class OP3Tracker():
    def __init__(self):
        rospy.init_node('op3_tracker')
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Initializing OP3 Tracker Joint Controller Node...")

        # from op3 following motion
        self.module_name = "direct_control_module" # MODULE_NAME
        # self.l_shoulder_3d_ = [0, 0, 0]
        self.is_ready = False
        self.is_playable = False
        # self.debug_print = False

        self.rate = rospy.get_param('~joint_controller_rate', 5)
        rate = rospy.Rate(self.rate)

        # The namespace may be set by the servo controller (usually null)
        namespace = rospy.get_namespace()

        self.joints = rospy.get_param(namespace + '/joints', '')
        self.skel_to_joint_map = rospy.get_param("~skel_to_joint_map", dict())
        self.use_real_robot = rospy.get_param('~use_real_robot', True) # False?
        self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.5)
        self.tracker_commands = rospy.get_param('~tracker_commands', ['STOP', 'TELEOP_JOINTS'])
        self.HALF_PI = pi / 2.0

        # Publisher
        op3_joint_pub = rospy.Publisher('/robotis/direct_control/set_joint_states', JointState, queue_size=0) # queue_size=5?
        dxl_torque_pub = rospy.Publisher('/robotis/dxl_torque', String, queue_size=0)
        set_module_pub = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=0)
        init_pose_pub = rospy.Publisher('/robotis/base/ini_pose', String, queue_size=0)

        # Service
        rospy.wait_for_service('/robotis/get_present_joint_ctrl_modules')
        try:
            self.get_module_clinet = rospy.ServiceProxy('/robotis/get_present_joint_ctrl_modules', GetJointModule)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # Subscribe to the skeleton topic.
        rospy.Subscriber('skeleton', Skeleton, self.skeleton_handler)
        rospy.Subscriber('/robotis/open_cr/button', String, self.button_handler)

        # Store the current skeleton configuration in a local dictionary.
        self.skeleton = dict()
        self.skeleton['confidence'] = dict()
        self.skeleton['position'] = dict()
        self.skeleton['orientation'] = dict()

        # Set up the tracker command service for this node.
        self.set_command = rospy.Service('~set_command', SetCommand, self.set_command_callback)

        # Use the joint_state publisher for a fake robot
        if not self.use_real_robot:
            self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=5)
        else:
            # Initialize servo controllers for a real robot
            self.init_servos()

        # The get_joints command parses a URDF description of the robot to get all the non-fixed joints.
        self.cmd_joints = PTL.get_joints()

        # Store the last joint command so we can stop and hold a given posture.
        self.last_cmd_joints = PTL.get_joints()

        # Initialize the robot in the stopped state.
        self.tracker_command = "STOP"

        while not rospy.is_shutdown():              
            # Execute the behavior appropriate for the current command.
            if self.tracker_command in self.tracker_commands:
                if self.tracker_command == 'STOP':
                    self.stop_joints()
                else:
                    self.teleop_joints()
                    self.cmd_joints.header.stamp = rospy.Time.now()

                if self.use_real_robot:
                    try:
                        self.pub_joint_cmds(self.cmd_joints)
                    except:
                        pass
                else:
                    self.joint_state = self.cmd_joints
                    self.joint_state_pub.publish(self.joint_state)
            else:
                pass

            self.last_cmd_joints = self.cmd_joints

            rate.sleep()
    def button_handler(self, msg):
        if (msg.data == "mode"):
            check_torque()
            handle_module()
            go_init_pose()

            self.is_playable = True
        elif (msg.data == "start"):
            handle_playing()
        elif (msg.data == "user"):
            set_base_init_pose()
            self.is_ready = False
            self.playable = False

    def check_torque():
        check_msg = String()
        check_msg.data = "check"

        

    def skeleton_handler(self, msg):
        for joint in msg.name:
            self.skeleton['confidence'][joint] = msg.confidence[msg.name.index(joint)]
            self.skeleton['position'][joint] = KDL.Vector(msg.position[msg.name.index(joint)].x,
                                                          msg.position[msg.name.index(joint)].y,
                                                          msg.position[msg.name.index(joint)].z)
            self.skeleton['orientation'][joint] = KDL.Rotation.Quaternion(msg.orientation[msg.name.index(joint)].x,
                                                                          msg.orientation[msg.name.index(joint)].y,
                                                                          msg.orientation[msg.name.index(joint)].z,
                                                                          msg.orientation[msg.name.index(joint)].w)
        # Last function
        def shutdown(self):
            rospy.loginfo('Shutting down Tracker Joint Controller Node.')

# Main function
if __name__ == '__main__':
    try:
        OP3Tracker()
    except rospy.ROSInterruptException:
        pass
