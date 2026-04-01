#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

# UR10 joint order
JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

def move_to_elbow_up():
    rospy.init_node('move_to_elbow_up')
    
    # Wait for action server
    client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for action server 'follow_joint_trajectory'...")
    client.wait_for_server()
    rospy.loginfo("Connected to action server.")

    # Get current joint state as starting point
    rospy.loginfo("Waiting for current joint_states...")
    joint_states = rospy.wait_for_message("joint_states", JointState)
    
    # joint_states.name and joint_states.position might be in a different order,
    # so we build a dictionary to extract the correct current positions safely.
    current_joints_dict = dict(zip(joint_states.name, joint_states.position))
    start_pos = [current_joints_dict[j] for j in JOINT_NAMES]
    
    # We define the specific joint angles to match the desired pose:
    # Tool position approx: X=0.0m, Y=0.5m, Z=0.5m
    # Configuration: Elbow UP, tool pointing down at the table.
    # (These values are typical rad angles for UR10 to achieve this).
    target_pos = [
        1.57,       # shoulder_pan: turn 90 deg left to face Y=0.5m
        -1.30,      # shoulder_lift: rotate back/up
        1.50,       # elbow: bend "up" and forward
        -1.77,      # wrist_1: point tool down towards the table
        -1.57,      # wrist_2: orient correctly
        0.0         # wrist_3: rotation around tool axis
    ]

    # Create the goal and trajectory
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    # Point 1: Current position at time 0
    g.trajectory.points.append(
        JointTrajectoryPoint(
            positions=start_pos,
            velocities=[0]*6,
            time_from_start=rospy.Duration(0.0)
        )
    )
    
    # Point 2: Target position reached after 4.0 seconds
    g.trajectory.points.append(
        JointTrajectoryPoint(
            positions=target_pos,
            velocities=[0]*6,
            time_from_start=rospy.Duration(4.0)
        )
    )

    rospy.loginfo("Sending trajectory goal to UR10 (Elbow UP configuration)...")
    client.send_goal(g)
    
    # Wait for it to finish
    client.wait_for_result()
    rospy.loginfo("Movement completed!")

if __name__ == '__main__':
    try:
        move_to_elbow_up()
    except rospy.ROSInterruptException:
        pass
