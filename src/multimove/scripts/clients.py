#!/usr/bin/env python3
import rospy
import actionlib
import smach
import time
import math

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from multimove.msg import joints_status
    
class PrepareStates(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes = ['success'], output_keys = ['joints', 'time'])
        
    def execute(self, userdata):

        userdata.joints = {
                        'joint_1': {'target': 3.0, 'dependency': None},
                        'joint_2': {'target': 0.0, 'dependency': None},
                        'joint_3': {'target': -1.8, 'dependency': {'joint': 'joint_1', 'percentage': 20}},
                        'joint_4': {'target': -1.2, 'dependency': {'joint': 'joint_1', 'percentage': 20}},
                        'joint_5': {'target': 0.2, 'dependency': {'joint': 'joint_4', 'percentage': 60}},
                        'joint_6': {'target': -0.2, 'dependency': {'joint': 'joint_4', 'percentage': 60}},
                        }
                    
        userdata.time = 5.0

        status_pub = rospy.Publisher('/multimove_simulation/joints_status', joints_status, queue_size=10)

        timeout = time.time() + 2
        while time.time() < timeout:
            status_pub.publish(joints_status())

        return 'success'

class Client(smach.State):

    def __init__(self, order):
        smach.State.__init__(self, outcomes = ['success', 'failure'], input_keys=['joints', 'time'])
        self.name = "Client" + str(order + 1)
        self.order = order
        
        self.client = actionlib.SimpleActionClient(f'/arm_robot/joint_{order}_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        self.wait = True

    def feedback_cb(self, feedback):
        # rospy.loginfo(f'{feedback.joint_names[0]} current position: {feedback.actual.positions[0]}')
        pass

    def done_cb(self, state, result):
        rospy.set_param(f'/multimove_simulation/finished/joint_{self.order}', True)
        rospy.loginfo(result)

    def execute(self, userdata):

        self.client.wait_for_server()

        goal = FollowJointTrajectoryGoal()

        trajectory = JointTrajectory(header = Header(stamp = rospy.Time.now()), joint_names = [f'joint_{self.order}'])
        
        targets = [userdata.joints[f'joint_{self.order}']['target']]
        velocities = [0.0]

        trajectory.points.append(JointTrajectoryPoint(positions = targets, velocities = velocities, time_from_start = rospy.Duration(secs = userdata.time)))

        goal.trajectory = trajectory

        while not rospy.get_param(f'/multimove_simulation/start/joint_{self.order}'):
            pass

        self.client.send_goal(goal, feedback_cb = self.feedback_cb, done_cb = self.done_cb)

        self.client.wait_for_result()

        return 'success'

class MonitorClient(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success'],  input_keys=['joints'])
        self.positions = {'joint_1': 0, 'joint_2': 0, 'joint_3': 0, 'joint_4': 0, 'joint_5': 0, 'joint_6': 0}

    def position_cb(self, data):
        for i in range(len(data.name)):
            self.positions[data.name[i]] = data.position[i]

    def execute(self, userdata):

        joints = userdata.joints
        
        rospy.Subscriber('/arm_robot/joint_states', JointState, self.position_cb)

        while not self.preempt_requested():
            for joint, data in joints.items():
                if (rospy.get_param(f'/multimove_simulation/start/{joint}') == False):
                    if data['dependency'] == None:
                        rospy.loginfo('Starting {joint}')
                        rospy.set_param(f'/multimove_simulation/start/{joint}', True)
                    else:
                        dependency_joint = data['dependency']['joint']
                        dependency_joint_target = joints[dependency_joint]['target']
                        dependency_perc = data['dependency']['percentage']
                        if abs(self.positions[dependency_joint]) > abs(dependency_perc * dependency_joint_target / 100):
                            rospy.loginfo('Starting {joint}')
                            rospy.set_param(f'/multimove_simulation/start/{joint}', True)

        rospy.loginfo(self.positions)
        return "success"

def child_termination_cb(outcome_map):
    if outcome_map['Client1'] and outcome_map['Client2'] and outcome_map['Client3'] and outcome_map['Client4'] and outcome_map['Client5'] and outcome_map['Client6']:
        return True

    return False

def outcome_cb(outcome_map):
    if outcome_map['Client1'] == 'success' and outcome_map['Client2'] == 'success' and outcome_map['Client3'] == 'success' and outcome_map['Client4'] == 'success' and outcome_map['Client5'] == 'success' and outcome_map['Client6'] == 'success':
        return 'success'
    else:
        return 'failure'

def main():
    sm = smach.StateMachine(outcomes = ['success', 'failure'])

    with sm:
        smach.StateMachine.add("PrepareStates", PrepareStates(), transitions={'success': 'Concurrent_States'})

        sm_concurrent = smach.Concurrence(
                                        outcomes = ['success', 'failure'],
                                        default_outcome = 'failure',
                                        child_termination_cb = child_termination_cb,
                                        outcome_cb = outcome_cb,
                                        input_keys = ['joints', 'time'])

        with sm_concurrent:
            for i in range (6):
                smach.Concurrence.add(f'Client{i + 1}', Client(i + 1))
                smach.Concurrence.add('MonitorClient', MonitorClient())
        smach.StateMachine.add('Concurrent_States', sm_concurrent, transitions={'success': 'success', 'failure': 'failure'})
    
    sm.execute()

if __name__ == '__main__':

    rospy.init_node("smach_joints_controller")
    main()