#!/usr/bin/python

"""
  Copyright (c) 2011 Vanadium Labs LLC. All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holders nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest('maxwell_teleop')
import rospy

from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

class HandHoldTeleop:
    def __init__(self):
        self.listener = TransformListener()
        self.rate = rospy.Rate(100)
        self.pub = rospy.Publisher('/cmd_vel', Twist)

        self.root = rospy.get_param('root_link','arm_link')
        self.tip = rospy.get_param('tip_link','gripper_link')

        self.x_home = rospy.get_param('x_home', 0.35)
        self.z_home = rospy.get_param('z_home', -0.2)
        self.x_rate = rospy.get_param('x_rate', 8.0)
        self.r_rate = rospy.get_param('r_rate', 5.0)

        rospy.loginfo("Started")        

        while not rospy.is_shutdown():
            gripper_pose = PoseStamped()
            gripper_pose.header.frame_id = self.tip
            
            try:
                pose = self.listener.transformPose(self.root, gripper_pose).pose
                if pose.position.z > self.z_home:
                    msg = Twist()
                    msg.linear.x = (pose.position.x - self.x_home) * self.x_rate
                    msg.angular.z = pose.position.y * self.r_rate
                    print msg.linear.x, msg.angular.z
                    self.pub.publish(msg)
                else:
                    self.pub.publish(Twist())
            except:
                pass
                #rospy.logerr("Transform failed")

            self.rate.sleep();

        self.pub.publish(Twist()) 

if __name__=="__main__":
    rospy.init_node("handhold_teleop")
    HandHoldTeleop()

