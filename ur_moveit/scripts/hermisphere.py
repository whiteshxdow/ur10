#!/usr/bin/env python3
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import numpy as np
import time
import sys

def distribute_points_on_hemisphere(radius, center, num_points=20):
    points = []
    for i in range(num_points):
        phi = np.arccos(1 - 2 * (i + 0.5) / num_points)
        theta = np.pi * (1 + 5**0.5) * (i + 0.5)
        x = center[0] + radius * np.cos(theta) * np.sin(phi)
        y = center[1] + radius * np.sin(theta) * np.sin(phi)
        z = center[2] + radius * np.cos(phi)
        points.append([x, y, z])
    return points

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10e_move_to_hemisphere_points', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "full_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Adjusted parameters for hemisphere point generation
    radius = 0.2  # Smaller radius to keep the points close
    center = [0.5, 0.0, 0.7]  # Adjusted center to elevate the hemisphere and keep it within reach

    points = distribute_points_on_hemisphere(radius, center, 20)

    for point in points:
        pose_goal = Pose()
        pose_goal.position.x = point[0]
        pose_goal.position.y = point[1]
        pose_goal.position.z = point[2]
        
        # Set the orientation to point downwards
        pose_goal.orientation.x = 1.0
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 0.0

        move_group.set_pose_target(pose_goal)
        
        # Attempt to move to the pose
        success = move_group.go(wait=True)
        move_group.stop()  # Calls stop() to ensure there is no residual movement
        move_group.clear_pose_targets()

        if not success:
            rospy.logwarn("Failed to reach pose: {}".format(pose_goal))
            continue

        rospy.sleep(2)  # Stay at the point for 3 seconds

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()

