#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np
import copy
import tf
from tf.transformations import euler_from_quaternion

def get_current_pose(tf_listener_):
    try:
        (trans, rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
        return trans, rot, rospy.Time.now()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.loginfo("TF Exception")
        return None, None, None

def calculate_angular_speed(prev_rot, current_rot, prev_time, current_time):
    if prev_rot is None or current_rot is None:
        return None
    
    # Convert quaternions to Euler angles
    prev_euler = euler_from_quaternion(prev_rot)
    current_euler = euler_from_quaternion(current_rot)
    
    # Calculate angular change in each axis
    delta_angles = np.array(current_euler) - np.array(prev_euler)
    delta_time = (current_time - prev_time).to_sec()

    if delta_time > 0:
        # Compute angular speed (rad/s)
        angular_speed = delta_angles / delta_time
        return angular_speed
    else:
        return None

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('tf_listener')

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"  # Change this to your group name
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    tf_listener_ = tf.TransformListener()
    rospy.sleep(1)
    
    prev_rot, prev_time = None, None
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x += 0.1  # Example position change
        wpose.position.y += 0.1
        waypoints.append(copy.deepcopy(wpose))

        # More waypoints can be added here
        
        # Compute the Cartesian path
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        # Set the duration for each segment of the trajectory
        durations = [5.0, 3.0]  # Example durations
        new_trajectory = moveit_msgs.msg.RobotTrajectory()
        new_trajectory.joint_trajectory = plan.joint_trajectory

        # Adjust time from start for each trajectory point
        for i, point in enumerate(new_trajectory.joint_trajectory.points):
            point.time_from_start = rospy.Duration(durations[min(i, len(durations)-1)])

        # Execute the planned trajectory
        move_group.execute(new_trajectory, wait=True)

        # Get current pose and calculate angular speed
        current_pose, current_rot, current_time = get_current_pose(tf_listener_)
        if prev_rot is not None:
            angular_speed = calculate_angular_speed(prev_rot, current_rot, prev_time, current_time)
            if angular_speed is not None and any(angular_speed > 0):
                rospy.loginfo("Angular Speed (rad/s): x: %f, y: %f, z: %f", *angular_speed)

        prev_rot, prev_time = current_rot, current_time
        rate.sleep()

if __name__ == '__main__':
    main()
