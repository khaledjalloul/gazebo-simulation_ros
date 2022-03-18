#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import math

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


if __name__ == '__main__':
    try:
        rospy.init_node('ros_control_python_publisher')
        publish()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass