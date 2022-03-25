#!/usr/bin/env python3
import rospy
import actionlib

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class ArmControl():

    def __init__(self):
        self.arm_client = actionlib.SimpleActionClient('/arm_robot/effort_joints_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.grip_client = actionlib.SimpleActionClient('/arm_robot/position_joints_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    def feedback_cb(self, feedback):
        rospy.loginfo(feedback)

    def done_cb(self, state, result):
        rospy.loginfo('result')
        rospy.loginfo(result)

    def move(self, trajectories):

        self.arm_client.wait_for_server()
        self.grip_client.wait_for_server()

        arm_goal = FollowJointTrajectoryGoal()
        grip_goal = FollowJointTrajectoryGoal()

        arm_trajectory = JointTrajectory(header = Header(stamp = rospy.Time.now()))
        grip_trajectory = JointTrajectory(header = Header(stamp = rospy.Time.now()))
        
        arm_trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        grip_trajectory.joint_names = ['joint_51', 'joint_52']

        velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        speed = 1

        for i, trajectory in enumerate(trajectories):

            arm_trajectory.points.append(JointTrajectoryPoint(positions = trajectory[:4], velocities = velocities[:4], time_from_start = rospy.Duration(secs = (i+1) * speed)))
            grip_trajectory.points.append(JointTrajectoryPoint(positions = trajectory[4:6], velocities = velocities[4:6], time_from_start = rospy.Duration(secs = (i+1) * speed)))


        arm_goal.trajectory = arm_trajectory
        grip_goal.trajectory = grip_trajectory

        self.arm_client.send_goal(arm_goal, feedback_cb = self.feedback_cb, done_cb = self.done_cb)
        self.grip_client.send_goal(grip_goal, feedback_cb = self.feedback_cb, done_cb = self.done_cb)

        self.arm_client.wait_for_result()
        self.grip_client.wait_for_result()

if __name__ == '__main__':

    try:
        rospy.init_node('ros_control_python_publisher')
        
        control = ArmControl()

        control.move([[2, 0.3, -1.0, -1.4, 0.0, 0.0]])

        while not rospy.is_shutdown():
            control.move([
                [2.0, 0, -1.5, -1.4, 0.4, -0.4],
                [2.0, 0, -1.5, -1.4, 0.1, -0.1],
                [2.0, 0.3, -1.0, -1.4, 0.1, -0.1],
                [2.8, 0.3, -1.0, -1.4, 0.1, -0.1],
                [2.8, -0.8, -0.5, -1.5, 0.1, -0.1],
                [2.8, -0.8, -0.5, -1.5, 0.4, -0.4],
                [2.8, 0.3, -1.0, -1.4, 0.0, 0.0],
                [2, 0.3, -1.0, -1.4, 0.0, 0.0]
            ])
        
    except rospy.ROSInterruptException:
        pass
