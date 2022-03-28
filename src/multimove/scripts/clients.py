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
        smach.State.__init__(self, outcomes = ['success'], output_keys = ['poses', 'time', 'deps'])
        
    def execute(self, userdata):

        userdata.poses = {'joint_1': 2.0,
                        'joint_2': 0,
                        'joint_3': -1.5,
                        'joint_4': -1.4,
                        'joint_5': 0.1,
                        'joint_6': -0.1
                        }

        userdata.deps = {'joint_1': None,
                        'joint_2': None,
                        'joint_3': {'joint': 'joint_1', 'percentage': 30},
                        'joint_4': {'joint': 'joint_1', 'percentage': 30},
                        'joint_5': {'joint': 'joint_1', 'percentage': 70},
                        'joint_6': {'joint': 'joint_1', 'percentage': 70}
                        }
                    
        userdata.time = 5.0

        status_pub = rospy.Publisher('/multimove_simulation/joints_status', joints_status, queue_size=10)

        timeout = time.time() + 2
        while time.time() < timeout:
            status_pub.publish(joints_status())

        rospy.set_param('/multimove_simulation/start/joint_3', True)
        return 'success'

class Client(smach.State):

    def __init__(self, order):
        smach.State.__init__(self, outcomes = ['success', 'failure'], input_keys=['poses', 'time'])
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
        
        # poses = [userdata.poses[self.order - 1] * math.pi / 180]
        poses = [userdata.poses[self.order - 1]]
        velocities = [0.0]

        trajectory.points.append(JointTrajectoryPoint(positions = poses, velocities = velocities, time_from_start = rospy.Duration(secs = userdata.time)))

        goal.trajectory = trajectory

        while not rospy.get_param(f'/multimove_simulation/start/joint_{self.order}'):
            pass

        self.client.send_goal(goal, feedback_cb = self.feedback_cb, done_cb = self.done_cb)

        self.client.wait_for_result()

        return 'success'

class MonitorClient(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success'],  input_keys=['poses', 'deps'])
        self.positions = {'joint_1': 0, 'joint_2': 0, 'joint_3': 0, 'joint_4': 0, 'joint_5': 0, 'joint_6': 0}

    def position_cb(self, data):
        for i in range(len(data.name)):
            self.positions[data.name[i]] = data.position[i]

    def execute(self, userdata):
        #while not (rospy.get_param('Client1_finished') and rospy.get_param('Client2_finished') and rospy.get_param("Client3_finished")):
        #    if rospy.get_param('Client1_status') > 0.3 and not rospy.get_param('Client2_start'):
        #        rospy.set_param('Client2_start', True)
        #    if rospy.get_param('Client1_status') > 0.7 and not rospy.get_param('Client3_start'):
        #        rospy.set_param('Client3_start', True)
        percs = userdata.percs
        targets = userdata.poses

        rospy.loginfo(self.positions)
        rospy.Subscriber('/arm_robot/joint_states', JointState, self.position_cb)
        while not self.preempt_requested():
            for i in range(len(targets)):
                if (rospy.get_param(f'/multimove_research/start/joint_{i}') == False):
                    if (percs[i] == 0):
                        rospy.set_param(f'/multimove_research/start/joint_{i}', True)
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
                                        input_keys = ['poses', 'time', 'deps'])

        with sm_concurrent:
            for i in range (6):
                smach.Concurrence.add(f'Client{i + 1}', Client(i + 1))
                smach.Concurrence.add('MonitorClient', MonitorClient())
        smach.StateMachine.add('Concurrent_States', sm_concurrent, transitions={'success': 'success', 'failure': 'failure'})
    
    sm.execute()

if __name__ == '__main__':

    rospy.init_node("smach_joints_controller")
    main()