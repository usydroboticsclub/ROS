#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    jksPreciousWords = data.data
    bookcount=0
    for word in jksPreciousWords.split():
        if word.isdigit():
            bookcount=word
    rospy.loginfo("Oh my god. Apparently JK rowling has sold {} books!".format(bookcount))
    
def listener():

    rospy.init_node('hpfan', anonymous=True)

    rospy.Subscriber("someChannelName", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
