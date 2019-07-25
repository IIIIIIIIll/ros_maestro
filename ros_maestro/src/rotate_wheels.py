#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def dispatcher():
        while not rospy.is_shutdown():
            channel = input("Enter the channel: ")  
            target = input("Enter the target speed/local: ")
            final_str = "0%s%s" %(channel,target)  #two digit channel(padded with 0 for laziness) + four digit target
            rospy.loginfo(final_str)
            pub.publish(final_str)

if __name__ == '__main__':
    rospy.init_node('dispatcher')
    pub = rospy.Publisher('maestro_command', String, queue_size=10)
    dispatcher()
    
