#!/usr/bin/env python

""" 
Start and stop robot movement using buttons.
"""

import roslib; roslib.load_manifest('stop_and_go')
import rospy

from arbotix_msgs.msg import Digital
from geometry_msgs.msg import Twist

class StopAndGo:
    state = 255
    stopped = True

    def buttonCb(self, msg):
        if msg.value == 0 and self.state != 0:
            self.state = msg.value
            self.stopped = not self.stopped
        elif msg.value != 0 and self.state == 0:
            self.state = msg.value            

    def cmdvelCb(self, msg):
        if not self.stoppped:
            self.pub.write(msg)

    def __init__(self):
        rospy.init_node('stop_and_go')

        self.pub = rospy.Publisher('/cmd_vel', Twist)
        rospy.Subscriber('/turtlebot_node/cmd_vel',Twist,self.cmdvelCb)
        rospy.Subscriber('/arbotix/green_button',Digital,self.buttonCb)

        rospy.spin()
            

if __name__ == "__main__":
    bt = StopAndGo()

