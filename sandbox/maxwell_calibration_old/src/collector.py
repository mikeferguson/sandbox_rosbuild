#!/usr/bin/env python

import roslib; roslib.load_manifest("maxwell_calibration")
import rospy

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import *
from maxwell_move_arm.srv import * 

# generate a sampling of head poses
DEGREES_45 = 0.785398163
head_poses = list() #[ ["head_pan_joint","head_tilt_joint"] ]
for i in range(5):
    head_poses.append( [1.57-DEGREES_45*i, 1.0] )
for i in range(5):
    head_poses.append( [1.57-DEGREES_45*i, DEGREES_45] )

# and then arm poses
arm_poses = [ [0.339, 0.276, -0.041, 0], 
              [0.423, 0.149, -0.041, 0],
              [0.453, 0.000, -0.041, 0], 
              [0.423, -0.149, -0.041, 0],
              [0.339, -0.276, -0.041, 0],
              [0.325, -0.261, 0.271, -0.5],
              [0.405, -0.141, 0.271, -0.5],
              [0.433, 0.000, 0.271, -0.5],
              [0.405, 0.141, 0.271, -0.5],
              [0.325, 0.261, 0.271, -0.5] ]

class CalibrationCollector:
    
    def __init__(self):
        # node!
        rospy.init_node("collector")
        self.collect_joints = False
        self.collect_points = False
        self.joints = None

        self.listener = tf.TransformListener()

        # input topics
        rospy.Subscriber("/joint_states", JointState, self.jointsCb)
        rospy.Subscriber("/camera/rgb/points", PointCloud2, self.pointsCb)

        # services to move arm
        rospy.wait_for_service('/move_arm/move')
        self.move_server = rospy.ServiceProxy('/move_arm/move', MoveArm) 
    
        # output topics
        self.head_pan_pub_ = rospy.Publisher("/head_pan_joint/command",Float64)
        self.head_tilt_pub_ = rospy.Publisher("/head_tilt_joint/command",Float64)

        self.joints_pub_ = rospy.Publisher("~joint_states", JointState)
        self.points_pub_ = rospy.Publisher("~points", PointCloud2)
    
    def spin(self, head_poses, arm_poses):

        # move arm out of way
        self.setArm( 0.06, 0.166, 0.171, 1.16 )

        # move head through poses
        # at each, wait.... then capture XYZRGB cloud, send to new topic
        #                   then capture jointstates, send to new topic  ... sync times
        for pose in head_poses:
            rospy.loginfo("Move to position (" + str(pose[0]) + "," + str(pose[1]) + ")")
            self.setHead(pose[0],pose[1])
            rospy.loginfo("Wait for settle")
            rospy.sleep(2.0)
            self.stamp = rospy.Time.now()
            rospy.loginfo("Capture cloud/joints")
            self.collect_joints = True
            #self.collect_points = True
            while self.collect_joints or self.collect_points:
                pass
                
        # set head
        self.setHead(0,DEGREES_45)
        
        # move arm through poses
        # at each, wait.... then capture XYZRGB cloud, send to new topic
        #                   then capture jointstates, send to new topic  ... sync times
        for pose in arm_poses:
            rospy.loginfo("Move to position (" + str(pose[0]) + "," + str(pose[1]) + "," + str(pose[2]) + "," + str(pose[3]) + ")")
            self.setArm(pose[0],pose[1],pose[2],pose[3])
            rospy.loginfo("Wait for settle")
            rospy.sleep(2.0)
            self.stamp = rospy.Time.now()
            rospy.loginfo("Capture cloud/joints")
            self.collect_joints = True
            #self.collect_points = True
            while self.collect_joints or self.collect_points:
                pass
    
    def setHead(self, pan, tilt):
        while self.joints == None:
            pass
        p = self.joints.position[self.joints.name.index("head_pan_joint")]
        t = self.joints.position[self.joints.name.index("head_tilt_joint")]
        while pan != p or tilt != t:
            if p > pan:
                p -= 0.01
                if pan > p:
                    p = pan
            else:
                p += 0.01
                if pan < p:
                    p = pan
            if t > tilt:
                t -= 0.01
                if tilt > t:
                    t = tilt
            else:
                t += 0.01
                if tilt < t:
                    t = tilt            
            self.head_pan_pub_.publish(Float64(p))
            self.head_tilt_pub_.publish(Float64(t))
            rospy.sleep(0.05)

    def setArm(self, x, y, z, r):
        # create request
        req = MoveArmRequest()  
        req.pose_stamped.header.frame_id = "torso_link"
        req.pose_stamped.pose.position.x = x
        req.pose_stamped.pose.position.y = y
        req.pose_stamped.pose.position.z = z
        q = quaternion_from_euler(0, r, 0.0, 'sxyz')
        req.pose_stamped.pose.orientation.x = q[0]
        req.pose_stamped.pose.orientation.y = q[1]
        req.pose_stamped.pose.orientation.z = q[2]
        req.pose_stamped.pose.orientation.w = q[3]
        # send request
        try:
            res = self.move_server(req)
            print res
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)
        # wait for movement to complete
        self.waitForGoalCompletion( [x,y,z,r] )

    def waitForGoalCompletion(self, goal, timeout=10.0):
        if goal != None:
            (gx,gy,gz,gpsi) = goal
            time = 0
            while time < timeout:
                try: 
                    ((x,y,z), rot) = self.listener.lookupTransform('torso_link', 'gripper_link', rospy.Time(0))
                    (phi, psi, theta) = tf.transformations.euler_from_quaternion(rot)
                    if abs(gx-x) < 0.02 and abs(gy-y) < 0.02 and abs(gz-z) < 0.02 and abs(gpsi-psi)<0.075:
                        return True
                except (tf.LookupException, tf.ConnectivityException):
                    rospy.logerr("Failed to transform gripper to torso link")
                    return False
                rospy.sleep(0.1)
                time += 0.1
            return False

    def jointsCb(self, msg):
        self.joints = msg
        if self.collect_joints: 
            msg.header.stamp = self.stamp
            self.joints_pub_.publish(msg)
            self.collect_joints = False

    def pointsCb(self, msg):
        if self.collect_points: 
            msg.header.stamp = self.stamp
            self.points_pub_.publish(msg)
            self.collect_points = False
        

if __name__=="__main__":
    collector = CalibrationCollector()
    collector.spin(head_poses,arm_poses)
    
