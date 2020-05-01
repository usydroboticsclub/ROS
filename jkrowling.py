#!/usr/bin/env python

import rospy

from std_msgs.msg import String

def node():
    rospy.init_node('jkrowling', anonymous=True)

    pub = rospy.Publisher('someChannelName', String, queue_size=10) # String is the datatype, and if the channel is busy then we'll hold up to 10 messages in our local buffer.
    sales = 0
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        pub.publish("Did you know I've sold {} Harry Potter books?".format(sales))
        sales = sales + 1
        rate.sleep()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass