#!/usr/bin/env python3
from typing import Mapping
import rospy
import actionlib
import smach

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from multimove.msg import joints_status
    
class PrepareStates(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes = ['success'], output_keys = ['trajectory'])
        
    def execute(self, userdata):

        userdata.trajectory = [{
                        'joint_1': {'target': 1.0, 'dependency': None},
                        'joint_2': {'target': 0.0, 'dependency': None},
                        'joint_3': {'target': -1.0, 'dependency': {'joint': 'joint_1', 'percentage': 40}},
                        'joint_4': {'target': -0.6, 'dependency': None},
                        'joint_5': {'target': 0.3, 'dependency': {'joint': 'joint_4', 'percentage': 70}},
                        'joint_6': {'target': -0.3, 'dependency': {'joint': 'joint_4', 'percentage': 70}},
                        }, 
                        {
                        'joint_1': {'target': 2.0, 'dependency': None},
                        'joint_2': {'target': 0.5, 'dependency': {'joint': 'joint_1', 'percentage': 40}},
                        'joint_3': {'target': -0.6, 'dependency': None},
                        'joint_4': {'target': -0.8, 'dependency': {'joint': 'joint_2', 'percentage': 30}},
                        'joint_5': {'target': 0.01, 'dependency': {'joint': 'joint_4', 'percentage': 70}},
                        'joint_6': {'target': -0.01, 'dependency': {'joint': 'joint_4', 'percentage': 70}},
                        },
                        {
                        'joint_1': {'target': 3.0, 'dependency': None},
                        'joint_2': {'target': 0.0, 'dependency': None},
                        'joint_3': {'target': -1.4, 'dependency': None},
                        'joint_4': {'target': -1.0, 'dependency': {'joint': 'joint_1', 'percentage': 30}},
                        'joint_5': {'target': 0.3, 'dependency': {'joint': 'joint_4', 'percentage': 70}},
                        'joint_6': {'target': -0.3, 'dependency': {'joint': 'joint_4', 'percentage': 70}},
                        }
                    ]

        return 'success'

class updateTrajectory(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes = ['success', 'done'], input_keys = ['trajectory'], output_keys = ['trajectory'])

    def execute(self, userdata):
        
        if len(userdata.trajectory) == 1:
            return 'done'

        else:
            newTrajectory = userdata.trajectory[1:]
            userdata.trajectory = newTrajectory

            for i in range(6):
                rospy.set_param(f'/multimove_simulation/start/joint_{i+1}', False)

            return 'success'
        

class Client(smach.State):

    def __init__(self, order):
        smach.State.__init__(self, outcomes = ['success', 'failure'], input_keys=['trajectory'], output_keys=['trajectory'])
        self.name = "Client" + str(order + 1)
        self.order = order
        
        self.client = actionlib.SimpleActionClient(f'/arm_robot/joint_{order}_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    def feedback_cb(self, feedback):
        # rospy.loginfo(f'{feedback.joint_names[0]} current position: {feedback.actual.positions[0]}')
        pass

    def done_cb(self, state, result):
        rospy.loginfo(f'{self.name} finished with result:\n{result}')


    def execute(self, userdata):

        self.client.wait_for_server()

        goal = FollowJointTrajectoryGoal()

        trajectory = JointTrajectory(header = Header(stamp = rospy.Time.now()), joint_names = [f'joint_{self.order}'])
        
        current_path = userdata.trajectory[0]
        targets = [current_path[f'joint_{self.order}']['target']]
        velocities = [0.0]

        trajectory.points.append(JointTrajectoryPoint(positions = targets, velocities = velocities, time_from_start = rospy.Duration(secs = 3)))

        goal.trajectory = trajectory

        while not rospy.get_param(f'/multimove_simulation/start/joint_{self.order}'):
            pass

        self.client.send_goal(goal, feedback_cb = self.feedback_cb, done_cb = self.done_cb)

        self.client.wait_for_result()

        return 'success'

class MonitorClient(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success', 'preempted'],  input_keys=['trajectory'])
        self.positions = {'joint_1': 0, 'joint_2': 0, 'joint_3': 0, 'joint_4': 0, 'joint_5': 0, 'joint_6': 0}

    def position_cb(self, data):
        for i in range(len(data.name)):
            self.positions[data.name[i]] = data.position[i]

    def execute(self, userdata):
        trajectory = userdata.trajectory[0]
        
        rospy.Subscriber('/arm_robot/joint_states', JointState, self.position_cb)

        for joint, data in trajectory.items():
            if (rospy.get_param(f'/multimove_simulation/start/{joint}') == False):
                if data['dependency'] == None:
                    rospy.loginfo(f'Starting {joint}')
                    rospy.set_param(f'/multimove_simulation/start/{joint}', True)
                else:
                    dependency_joint = data['dependency']['joint']
                    dependency_joint_target = trajectory[dependency_joint]['target']
                    dependency_perc = data['dependency']['percentage']
                    if abs(self.positions[dependency_joint]) > abs(dependency_perc * dependency_joint_target / 100):
                        rospy.loginfo(f'Starting {joint}')
                        rospy.set_param(f'/multimove_simulation/start/{joint}', True)
        rospy.sleep(0.5)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'success'

def child_termination_cb(outcome_map):
    if outcome_map['Client1'] and outcome_map['Client2'] and outcome_map['Client3'] and outcome_map['Client4'] and outcome_map['Client5'] and outcome_map['Client6']:
        return True

    return False

def outcome_cb(outcome_map):
    if outcome_map['Client1'] == 'success' and outcome_map['Client2'] == 'success' and outcome_map['Client3'] == 'success' and outcome_map['Client4'] == 'success' and outcome_map['Client5'] == 'success' and outcome_map['Client6'] == 'success':
        return 'successOutcome'
    else:
        return 'failureOutcome'

def main():
    sm = smach.StateMachine(outcomes = ['SUCCESS', 'FAILURE'])

    with sm:
        smach.StateMachine.add("PrepareStates", PrepareStates(), {'success': 'Concurrent_States'})

        smach.StateMachine.add("updateTrajectory", updateTrajectory(), {'success': 'Concurrent_States', 'done': 'SUCCESS'})

        sm_concurrent = smach.Concurrence(
                                        outcomes = ['successOutcome', 'failureOutcome'],
                                        default_outcome = 'failureOutcome',
                                        child_termination_cb = child_termination_cb,
                                        outcome_cb = outcome_cb,
                                        input_keys = ['trajectory'])

        sm_monitor = smach.StateMachine(outcomes = ['monitor_success'], input_keys=['trajectory'])

        with sm_monitor:
            smach.StateMachine.add('MonitorClient', MonitorClient(), {'success': 'MonitorClient', 'preempted': 'monitor_success'})

        with sm_concurrent:
            for i in range (6):
                smach.Concurrence.add(f'Client{i + 1}', Client(i + 1))
                
            smach.Concurrence.add('MonitorStateMachine', sm_monitor)

        smach.StateMachine.add('Concurrent_States', sm_concurrent, {'successOutcome': 'updateTrajectory', 'failureOutcome': 'FAILURE'})
    
    sm.execute()

if __name__ == '__main__':

    rospy.init_node("smach_joints_controller")
    main()