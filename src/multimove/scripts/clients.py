import rospy
import actionlib
import smach
import time
import math

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from multimove.msg import joints_status
    
class PrepareStates(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes = ['success'], output_keys = ['poses', 'times'])
        
    def execute(self, userdata):

        userdata.poses = [0, 0, 0.0]
        userdata.times = [0, 0, 4]
        
        status_pub = rospy.Publisher('/multimove_simulation/joints_status', joints_status, queue_size=10)

        timeout = time.time() + 2
        while time.time() < timeout:
            status_pub.publish(joints_status())

        return 'success'

class Client(smach.State):

    def __init__(self, order, jointType):
        smach.State.__init__(self, outcomes = ['success', 'failure'], input_keys=['poses', 'times'])
        self.name = "Client" + str(order + 1)
        self.order = order
        self.jointType = jointType
        if (jointType == 'arm'):
            self.client = actionlib.SimpleActionClient('/arm_robot/effort_joints_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        else:
            self.client = actionlib.SimpleActionClient('/arm_robot/position_joints_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        self.wait = True

    def feedback_cb(self, feedback):
        rospy.loginfo(feedback)

    def done_cb(self, state, result):
        rospy.set_param('/multimove_simulation/finished/joint' + str(self.order), True)
        rospy.loginfo(result)

    def execute(self, userdata):

        self.client.wait_for_server()

        goal = FollowJointTrajectoryGoal()

        trajectory = JointTrajectory(header = Header(stamp = rospy.Time.now()))
        
        if (self.jointType == 'arm'):
            trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
            poses = [0.0, 0.0, 0.0, 0.0]
            poses[self.order] = userdata.poses[self.order] * math.pi / 180
            velocities = [0.0, 0.0, 0.0, 0.0]

        else:
            trajectory.joint_names = ['joint_51', 'joint_52']
            poses = [0.0, 0.0]
            poses[self.order - 4] = userdata.poses[self.order] * math.pi / 180
            velocities = [0.0, 0.0]

        print(userdata.times[self.order])
        trajectory.points.append(JointTrajectoryPoint(positions = poses, velocities = velocities, time_from_start = rospy.Duration(secs = userdata.times[self.order])))

        goal.trajectory = trajectory

        while not rospy.get_param('/multimove_simulation/start/joint' + str(self.order)):
            pass

        self.client.send_goal(goal, feedback_cb = self.feedback_cb, done_cb = self.done_cb)

        self.client.wait_for_result()

        return 'success'

class MonitorClient(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success'])

    def execute(self, userdata):
        while not (rospy.get_param('Client1_finished') and rospy.get_param('Client2_finished') and rospy.get_param("Client3_finished")):
            if rospy.get_param('Client1_status') > 0.3 and not rospy.get_param('Client2_start'):
                rospy.set_param('Client2_start', True)
            if rospy.get_param('Client1_status') > 0.7 and not rospy.get_param('Client3_start'):
                rospy.set_param('Client3_start', True)
        return "success"

def main():
    sm = smach.StateMachine(outcomes = ['success', 'failure'])

    with sm:
        smach.StateMachine.add("PrepareStates", PrepareStates(), transitions={'success': 'Concurrent_States'})

        sm_concurrent = smach.Concurrence(
                                        outcomes = ['success', 'failure'],
                                        default_outcome = 'failure',
                                        outcome_map = {'success': {'Client3': 'success'}},
                                        input_keys = ['poses', 'times'])

        with sm_concurrent:
            smach.Concurrence.add('Client3', Client(2, 'arm'))
            # smach.Concurrence.add('MonitorClient', MonitorClient())
        smach.StateMachine.add('Concurrent_States', sm_concurrent, transitions={'success': 'success', 'failure': 'failure'})
    
    sm.execute()

if __name__ == '__main__':

    rospy.init_node("gazebo_controller_clients")
    main()