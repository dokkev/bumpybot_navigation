#!/usr/bin/env python3
"""

This script is designed to work alongside the AMCL localization package. It is meant to be launched alongside the AMCL node in the navigation stack.
Its primary function is to continuously monitor the robot's current global pose and save the last known pose to a YAML file.
This saved pose can be used for re-initializing the robot's position upon restart of the navigation stack. 

"""


import rospy
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped
import os
import yaml
import math
class PoseSaver:
    def __init__(self):
        rospy.init_node('pose_saver', anonymous=True)

        # Path to save the YAML file
        rospack = rospkg.RosPack()
        package_name = rospy.get_param('~package_name', 'bumpybot_navigation') 
        package_path = rospack.get_path(package_name)
        self.pose_file = os.path.join(package_path, 'config', 'last_known_pose.yaml')

        # Last known pose
        self.last_pose = None

        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        rospy.loginfo("Pose Saver initialized. Saving to: %s", self.pose_file)

    def pose_callback(self, msg):
        self.last_pose = {
            '/amcl/initial_pose_x': msg.pose.pose.position.x,
            '/amcl/initial_pose_y': msg.pose.pose.position.y,
            '/amcl/initial_pose_a': 2*math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        }

    def save_pose_to_yaml(self):
        if self.last_pose is not None:
            try:
                with open(self.pose_file, 'w') as yaml_file:
                    yaml.dump(self.last_pose, yaml_file, default_flow_style=False)
                rospy.loginfo_once("Saved pose to YAML: %s", self.pose_file)
            except IOError as e:
                rospy.logerr("Failed to write pose to file: %s", e)

    def run(self):
        rate = rospy.Rate(0.5)  # 2 Hz
        while not rospy.is_shutdown():
            self.save_pose_to_yaml()  # Save the pose at 2 Hz
            rate.sleep()

if __name__ == '__main__':
    try:
        saver = PoseSaver()
        saver.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose Saver terminated.")
