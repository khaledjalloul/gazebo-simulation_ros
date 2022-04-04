#!/usr/bin/env python3
from typing import Mapping
from genpy import Duration
import rospy
import actionlib
import smach
import time
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

class PrepareStates(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes = ['success'], output_keys = ['trajectory'])
        self.unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        
    def execute(self, userdata):

        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause_gazebo()
        
        userdata.trajectory = [{
                        'joint_1':{'target': 1.0, 'dependency': None},
                        'joint_2': {'target': 0.0, 'dependency': None},
                        'joint_3': {'target': -1.0, 'dependency': {'joint': 'joint_1', 'percentage': 40}},
                        'joint_4': {'target': -0.6, 'dependency': None},
                        'joint_5': [
                            {'target': 0.4, 'dependency': {'joint': 'joint_3', 'percentage': 35}, 'duration': 1.2},
                            {'target': 0.01, 'dependency': None, 'duration': 1.2},
                            {'target': 0.4, 'dependency': None, 'duration': 1.2}],
                        'joint_6': [
                            {'target': -0.4, 'dependency': {'joint': 'joint_3', 'percentage': 35}, 'duration': 1.2},
                            {'target': -0.01, 'dependency': None, 'duration': 1.2},
                            {'target': -0.4, 'dependency': None, 'duration': 1.2}],
                        }, 
                        {
                        'joint_1': {'target': 2.0, 'dependency': None},
                        'joint_2': {'target': 0.5, 'dependency': {'joint': 'joint_1', 'percentage': 40}, 'duration': 3.5},
                        'joint_3': {'target': -0.6, 'dependency': None},
                        'joint_4': {'target': -0.8, 'dependency': {'joint': 'joint_2', 'percentage': 30}, 'duration': 2.5},
                        'joint_5': {'target': 0.01, 'dependency': {'joint': 'joint_3', 'percentage': 70}, 'duration': 1},
                        'joint_6': {'target': -0.01, 'dependency': {'joint': 'joint_3', 'percentage': 70}, 'duration': 1},
                        },
                        {
                        'joint_1': {'target': 3.0, 'dependency': None},
                        'joint_2': {'target': 0.0, 'dependency': None},
                        'joint_3': {'target': -1.4, 'dependency': None},
                        'joint_4': {'target': -1.0, 'dependency': {'joint': 'joint_1', 'percentage': 30}},
                        'joint_5': {'target': 0.3, 'dependency': {'joint': 'joint_3', 'percentage': 70}, 'duration': 1.5},
                        'joint_6': {'target': -0.3, 'dependency': {'joint': 'joint_3', 'percentage': 70}, 'duration': 1.5},
                        },
                    ]

        return 'success'

class updateTotalTrajectory(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes = ['success', 'done'], input_keys = ['trajectory'], output_keys = ['trajectory'])
        self.pause_gazebo = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

    def execute(self, userdata):
        
        if len(userdata.trajectory) == 1:
            rospy.wait_for_service('/gazebo/pause_physics')
            self.pause_gazebo()
            return 'done'

        else:
            newTrajectory = userdata.trajectory[1:]
            userdata.trajectory = newTrajectory

            for i in range(6):
                rospy.set_param(f'/multimove_simulation/start/joint_{i+1}', False)

            return 'success'
        
class updateTrajectory(smach.State):

    def __init__(self, order):
        smach.State.__init__(self, outcomes = ['success', 'done'], input_keys=['trajectory'], output_keys=['trajectory'])
        self.order = order

    def execute(self, userdata):
        current_path = userdata.trajectory[0]
        targets = current_path[f'joint_{self.order}']

        if len(targets) == 1:
            return 'done'

        else:
            new_targets = targets[1:]
            userdata.trajectory[0][f'joint_{self.order}'] = new_targets
            return 'success'

class Client(smach.State):

    def __init__(self, order):
        smach.State.__init__(self, outcomes = ['success', 'updateTrajectory'], input_keys=['trajectory'])
        self.name = "Client" + str(order)
        self.order = order
        
        self.client = actionlib.SimpleActionClient(f'/arm_robot/joint_{order}_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    def feedback_cb(self, feedback):
        # rospy.loginfo(f'{feedback.joint_names[0]} current position: {feedback.actual.positions[0]}')
        pass

    def done_cb(self, state, result):
        rospy.loginfo(f'{self.name} finished with result:\n{result}')


    def execute(self, userdata):

        self.client.wait_for_server()

        default_duration = 5.0
        targets = userdata.trajectory[0][f'joint_{self.order}']

        if str(type(targets)) == "<class 'list'>":
            target = targets[0]
        else:
            target = targets
    
        try:
            duration = rospy.Duration.from_sec(target['duration'])
        except KeyError:
            duration = rospy.Duration.from_sec(default_duration)

        while not rospy.get_param(f'/multimove_simulation/start/joint_{self.order}'):
            pass

        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory(header = Header(stamp = rospy.Time.now()), joint_names = [f'joint_{self.order}'])
        trajectory.points.append(JointTrajectoryPoint(positions = [target['target']], velocities = [0.0], time_from_start = duration))
        goal.trajectory = trajectory

        self.client.send_goal(goal, feedback_cb = self.feedback_cb, done_cb = self.done_cb)

        self.client.wait_for_result()

        if str(type(targets)) == "<class 'list'>":
            return 'updateTrajectory'

        return 'success'

class MonitorClient(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success', 'preempted'],  input_keys=['trajectory'])
        self.positions = {'joint_1': 0, 'joint_2': 0, 'joint_3': 0, 'joint_4': 0, 'joint_5': 0, 'joint_6': 0}
        self.prevPositions = {'joint_1': 0, 'joint_2': 0, 'joint_3': 0, 'joint_4': 0, 'joint_5': 0, 'joint_6': 0}

    def position_cb(self, data):
        for i in range(len(data.name)):
            self.positions[data.name[i]] = data.position[i]

    def execute(self, userdata):
        trajectory = userdata.trajectory[0]
        
        rospy.Subscriber('/arm_robot/joint_states', JointState, self.position_cb)

        for joint, data in trajectory.items():
            if str(type(data)) == "<class 'list'>":
                data = data[0]
            if (rospy.get_param(f'/multimove_simulation/start/{joint}') == False):
                if data['dependency'] == None:
                    rospy.loginfo(f'Starting {joint}')
                    self.prevPositions[joint] = self.positions[joint]
                    rospy.set_param(f'/multimove_simulation/start/{joint}', True)

                else:
                    dependency_joint = data['dependency']['joint']
                    if rospy.get_param(f'/multimove_simulation/start/{dependency_joint}'): # Wait until the dependency joint starts before monitoring it
                        dependency_joint_target = trajectory[dependency_joint]['target']
                        dependency_perc = data['dependency']['percentage']

                        if abs(self.positions[dependency_joint] - self.prevPositions[dependency_joint]) > abs(dependency_perc * (dependency_joint_target - self.prevPositions[dependency_joint]) / 100):
                            rospy.loginfo(f'Starting {joint}')
                            self.prevPositions[joint] = self.positions[joint]
                            rospy.set_param(f'/multimove_simulation/start/{joint}', True)
        rospy.sleep(0.2)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'success'

def child_termination_cb(outcome_map):
    if outcome_map['Client1StateMachine'] and outcome_map['Client2StateMachine'] and outcome_map['Client3StateMachine'] and outcome_map['Client4StateMachine'] and outcome_map['Client5StateMachine'] and outcome_map['Client6StateMachine']:
        return True

    return False

def outcome_cb(outcome_map):
    if outcome_map['Client1StateMachine'] == 'success' and outcome_map['Client2StateMachine'] == 'success' and outcome_map['Client3StateMachine'] == 'success' and outcome_map['Client4StateMachine'] == 'success' and outcome_map['Client5StateMachine'] == 'success' and outcome_map['Client6StateMachine'] == 'success':
        return 'successOutcome'
    else:
        return 'failureOutcome'

def main():
    sm = smach.StateMachine(outcomes = ['SUCCESS', 'FAILURE'])

    with sm:
        smach.StateMachine.add("PrepareStates", PrepareStates(), {'success': 'Concurrent_States'})

        smach.StateMachine.add("updateTotalTrajectory", updateTotalTrajectory(), {'success': 'Concurrent_States', 'done': 'SUCCESS'})

        sm_concurrent = smach.Concurrence(
                                        outcomes = ['successOutcome', 'failureOutcome'],
                                        default_outcome = 'failureOutcome',
                                        child_termination_cb = child_termination_cb,
                                        outcome_cb = outcome_cb,
                                        input_keys = ['trajectory'])

        sm_monitor = smach.StateMachine(outcomes = ['monitor_success'], input_keys=['trajectory'])

        sm_client1 = smach.StateMachine(outcomes = ['success'], input_keys=['trajectory'])

        with sm_client1:
            smach.StateMachine.add("Client1", Client(1), {'updateTrajectory': 'updateClient1Trajectory', 'success': 'success'})
            smach.StateMachine.add("updateClient1Trajectory", updateTrajectory(1), {'success': 'Client1', 'done': 'success'})

        sm_client2 = smach.StateMachine(outcomes = ['success'], input_keys=['trajectory'])

        with sm_client2:
            smach.StateMachine.add("Client2", Client(2), {'updateTrajectory': 'updateClient2Trajectory', 'success': 'success'})
            smach.StateMachine.add("updateClient2Trajectory", updateTrajectory(2), {'success': 'Client2', 'done': 'success'})

        sm_client3 = smach.StateMachine(outcomes = ['success'], input_keys=['trajectory'])

        with sm_client3:
            smach.StateMachine.add("Client3", Client(3), {'updateTrajectory': 'updateClient3Trajectory', 'success': 'success'})
            smach.StateMachine.add("updateClient3Trajectory", updateTrajectory(3), {'success': 'Client3', 'done': 'success'})

        sm_client4 = smach.StateMachine(outcomes = ['success'], input_keys=['trajectory'])

        with sm_client4:
            smach.StateMachine.add("Client4", Client(4), {'updateTrajectory': 'updateClient4Trajectory', 'success': 'success'})
            smach.StateMachine.add("updateClient4Trajectory", updateTrajectory(4), {'success': 'Client4', 'done': 'success'})

        sm_client5 = smach.StateMachine(outcomes = ['success'], input_keys=['trajectory'])

        with sm_client5:
            smach.StateMachine.add("Client5", Client(5), {'updateTrajectory': 'updateClient5Trajectory', 'success': 'success'})
            smach.StateMachine.add("updateClient5Trajectory", updateTrajectory(5), {'success': 'Client5', 'done': 'success'})

        sm_client6 = smach.StateMachine(outcomes = ['success'], input_keys=['trajectory'])

        with sm_client6:
            smach.StateMachine.add("Client6", Client(6), {'updateTrajectory': 'updateClient6Trajectory', 'success': 'success'})
            smach.StateMachine.add("updateClient6Trajectory", updateTrajectory(6), {'success': 'Client6', 'done': 'success'})

        with sm_monitor:
            smach.StateMachine.add('MonitorClient', MonitorClient(), {'success': 'MonitorClient', 'preempted': 'monitor_success'})

        with sm_concurrent:
            smach.Concurrence.add('Client1StateMachine', sm_client1)
            smach.Concurrence.add('Client2StateMachine', sm_client2)
            smach.Concurrence.add('Client3StateMachine', sm_client3)
            smach.Concurrence.add('Client4StateMachine', sm_client4)
            smach.Concurrence.add('Client5StateMachine', sm_client5)
            smach.Concurrence.add('Client6StateMachine', sm_client6)
            smach.Concurrence.add('MonitorStateMachine', sm_monitor)

        smach.StateMachine.add('Concurrent_States', sm_concurrent, {'successOutcome': 'updateTotalTrajectory', 'failureOutcome': 'FAILURE'})
    
    sm.execute()

if __name__ == '__main__':

    rospy.init_node("smach_joints_controller")
    main()