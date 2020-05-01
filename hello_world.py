#!/usr/bin/env python
# ^ the above line is called a `shebang`, and it just tells the shell to use python as an interpreter.

import rospy
# rospy is a package, just like anything else!

def node():
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo("I'm alive!")
        rate.sleep()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass