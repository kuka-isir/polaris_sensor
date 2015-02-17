#!/usr/bin/env python
'''
Created on Feb 17, 2015

@author: Antoine Hoarau <hoarau.robotics@gmail.com>
'''

import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PointStamped
import sys
import tf

class Listener:
    def __init__(self):
        rospy.init_node("tf_listener",anonymous=True)

        self.tf_frame = "/lbr4_7_link"
        if rospy.has_param('~tf_frame'):
            self.tf_frame = rospy.get_param('~tf_frame')
        else:
            rospy.logwarn("tf_frame argument not set, using default "+str(self.tf_frame))

        self.base_frame = "/base_link"
        if rospy.has_param('~base_frame'):
            self.base_frame = rospy.get_param('~base_frame')
        else:
            rospy.logwarn("base_frame argument not set, using default "+self.base_frame)
            
        self.topic_out = "/pt_out"
        if rospy.has_param('~topic_out'):
            self.topic_out = rospy.get_param('~topic_out')
        else:
            rospy.logwarn("topic_out argument not set, using default "+self.topic_out)

        rospy.loginfo("Listening to frame "+self.tf_frame+" (base "+self.base_frame+")")
        rospy.loginfo("Publishing to "+self.topic_out)
        
        self.pt_pub = rospy.Publisher(self.topic_out,PointStamped)
        
        self.tf = tf.TransformListener()

    def start(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                self.tf.waitForTransform(self.base_frame,self.tf_frame, rospy.Time(0), rospy.Duration(10.0) )
                translation,rotation = self.tf.lookupTransform(self.base_frame,self.tf_frame, rospy.Time(0))
                pt_out = PointStamped()
                pt_out.header.frame_id =  self.base_frame
                pt_out.header.stamp = rospy.Time.now()
                pt_out.point.x = translation[0]
                pt_out.point.y = translation[1]
                pt_out.point.z = translation[2]
                self.pt_pub.publish(pt_out)
            except Exception,e:
                rospy.logerr(e)
            r.sleep()

def main(argv):
    L = Listener()
    L.start()

if __name__ == '__main__':
    main(sys.argv)
    exit(0)


