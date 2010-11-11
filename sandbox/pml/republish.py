#!/usr/bin/env python

"""
  Republish a laser scan topic, with interpolation and other crap
  Copyright (c) 2008-2010 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
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

import roslib; roslib.load_manifest('arbotix')
import rospy

from sensor_msgs.msg import LaserScan

class republish:

    def __init__(self):
        rospy.init_node("re_scan")
        rospy.Subscriber("base_scan", LaserScan, self.laserCb)
        self.scanPub = rospy.Publisher('scan', LaserScan)
        rospy.spin()

    def laserCb(self, msg):
        n_ranges = list()
        pv = msg.ranges[0]
        for i in range(len(msg.ranges)):
            if pv>0.0 and msg.ranges[i]>0.0:
                n_ranges.append((pv+msg.ranges[i])/2)
            else:
                n_ranges.append(0.0)
            n_ranges.append(msg.ranges[i])
            pv = msg.ranges[i]
        msg.ranges = n_ranges
        msg.angle_increment = msg.angle_increment/2
        msg.time_increment = 1.5/59
        self.scanPub.publish(msg)

if __name__ == "__main__":
    a = republish()

