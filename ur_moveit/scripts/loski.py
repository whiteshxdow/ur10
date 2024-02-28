#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

def move_to_named_pose(pose_name):
    # Note: Initializing the moveit_commander outside the function now
    group_name = "full_arm"  # Updated group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_named_target(pose_name)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_named_pose_loop', anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    poses = ["home", "straight_up", "loop_pose_1" , "loop_pose_2" , "loop_pose_3", "loop_pose_4" ]  # List of poses to iterate through
    try:
        while not rospy.is_shutdown():  # Check for ROS shutdown signal
            for pose_name in poses:
                print(f"Moving to {pose_name} pose")
                move_to_named_pose(pose_name)
                rospy.sleep(2)  # Wait for 2 seconds between poses for visibility
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Execution interrupted by the user.")
    finally:
        moveit_commander.roscpp_shutdown()

