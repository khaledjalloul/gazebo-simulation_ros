#!/usr/bin/env python3
import rospy
import actionlib
import math

from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal

def joint(num):
    return f'/ros_control_tester/joint{num}_position_controller/command'

def publish():
    pub1 = rospy.Publisher(joint(1), Float64, queue_size=10)
    pub2 = rospy.Publisher(joint(2), Float64, queue_size=10)
    pub3 = rospy.Publisher(joint(3), Float64, queue_size=10)
    r = rospy.Rate(20)
    i = 0.0
    
    while not rospy.is_shutdown():
        pub1.publish(math.sin(i))
        pub2.publish(-0.7)
        pub3.publish(math.sin(i))
        i += 0.05
        r.sleep()

def topic_trajectory():
    pub = rospy.Publisher('/ros_control_tester/joint3_trajectory_controller/command', JointTrajectory, queue_size=10)

    trajectory = JointTrajectory()

    trajectory.header = Header(stamp = rospy.Time.now())
    trajectory.joint_names = ['leg_arm_joint']

    points = []
    points.append(JointTrajectoryPoint(positions = [0.0], velocities = [1.0], time_from_start = rospy.Duration(secs = 1)))
    points.append(JointTrajectoryPoint(positions = [1.0], velocities = [1.0], time_from_start = rospy.Duration(secs = 3)))
    points.append(JointTrajectoryPoint(positions = [1.5], velocities = [1.0], time_from_start = rospy.Duration(secs = 5)))
    points.append(JointTrajectoryPoint(positions = [2.5], velocities = [1.0], time_from_start = rospy.Duration(secs = 7)))
    trajectory.points = points

    while pub.get_num_connections() < 1:
        pass
    
    pub.publish(trajectory)


def feedback_cb(feedback):
    rospy.loginfo(feedback)

def done_cb(state, result):
    rospy.loginfo('result')
    rospy.loginfo(result)

def action_trajectory():

    client = actionlib.SimpleActionClient('/ros_control_tester/joints_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()

    trajectory = JointTrajectory()

    trajectory.header = Header(stamp = rospy.Time.now())
    trajectory.joint_names = ['toe_foot_joint', 'foot_leg_joint', 'leg_arm_joint']

    points = []
    points.append(JointTrajectoryPoint(positions = [1.0, 0.0, 1.0], velocities=[0.0, 0.0, 0.0], time_from_start = rospy.Duration(secs = 2)))
    points.append(JointTrajectoryPoint(positions = [1.0, 0.7, 2.0], velocities=[0.0, 0.0, 0.0], time_from_start = rospy.Duration(secs = 4)))
    points.append(JointTrajectoryPoint(positions = [1.0, 0.7, 2.0], velocities=[0.0, 0.0, 0.0], time_from_start = rospy.Duration(secs = 5)))
    points.append(JointTrajectoryPoint(positions = [1.0, 0.3, 2.0], velocities=[0.0, 0.0, 0.0], time_from_start = rospy.Duration(secs = 7)))
    points.append(JointTrajectoryPoint(positions = [2.0, 0.3, 2.0], velocities=[0.0, 0.0, 0.0], time_from_start = rospy.Duration(secs = 9)))
    points.append(JointTrajectoryPoint(positions = [2.0, 0.7, 2.0], velocities=[0.0, 0.0, 0.0], time_from_start = rospy.Duration(secs = 11)))
    points.append(JointTrajectoryPoint(positions = [2.0, 0.7, 2.0], velocities=[0.0, 0.0, 0.0], time_from_start = rospy.Duration(secs = 12)))
    points.append(JointTrajectoryPoint(positions = [2.0, 0.3, 2.0], velocities=[0.0, 0.0, 0.0], time_from_start = rospy.Duration(secs = 14)))
    points.append(JointTrajectoryPoint(positions = [0.0, 0.0, 0.0], velocities=[0.0, 0.0, 0.0], time_from_start = rospy.Duration(secs = 18)))

    trajectory.points = points

    goal.trajectory = trajectory

    client.send_goal(goal, feedback_cb = feedback_cb, done_cb = done_cb)
    client.wait_for_result()

if __name__ == '__main__':
    try:
        rospy.init_node('ros_control_python_publisher')
        # publish()
        action_trajectory()
    except rospy.ROSInterruptException:
        pass
