#!/usr/bin/env python

# useful robots executive

import roslib; roslib.load_manifest("useful_robots")
import rospy

from geometry_msgs.msg import PointStamped


class UsefulPlanner:
    """ A planner that does useful stuff? """

    in_gripper = ""  # nothing in the gripper
    
    def __init__(self):
        
        
        # 
        rospy.Subscriber("/commandPoint", self.pointCb)


    def pointCb(self, msg):
        """ Turn a geometry_msgs/PoseStamped into a command. 
            Orientation of pose should be normal to point surface. """
        
        # transform to base_link
        # extract (x, y, z) (r, p, y)

        # is point on floor?
        if z < 0.15:
            if abs(r) < 0.15 and abs(p) < 0.15:
                # drive to point  
    
            elif self.in_gripper == "":
                # pick up object on floor if gripper empty

            else:
                rospy.loginfo("Point near floor, but no possible actions")

        # is point on wall?
        elif ??:
            if self.in_gripper == "":
                
            else:
                rospy.loginfo("Point on wall, but no possible actions")
        else:

        

    

if __name__=="__main__":
    rospy.init_node("useful_robot_server")
    planner = UsefulPlanner()
    rospy.spin()

