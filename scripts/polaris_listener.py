#!/usr/bin/env python
'''
Created on Dec 9, 2014

@author: Antoine Hoarau <hoarau.robotics@gmail.com>
'''

import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PointStamped
import sys
import math

class PolarisListener:
    def __init__(self):
        rospy.init_node("polaris_listener",anonymous=True)

        self.tool_id = 0
        if rospy.has_param('~tool_id'):
            self.tool_id = int(rospy.get_param('~tool_id'))
        else:
            rospy.logwarn("tool_id argument not set, using default "+str(self.tool_id))

        self.topic_out = "/polaris_sensor/one_target"
        if rospy.has_param('~topic_out'):
            self.topic_out = rospy.get_param('~topic_out')
        else:
            rospy.logwarn("topic_out argument not set, using default "+self.topic_out)
            
        self.polaris_topic = "/polaris_sensor/targets"
        if rospy.has_param('~polaris_topic'):
            self.polaris_topic = rospy.get_param('~polaris_topic')
        else:
            rospy.logwarn("polaris_topic argument not set, using default "+self.polaris_topic)

        rospy.loginfo("Listening to tool_id "+str(self.tool_id)+" inside topic "+self.polaris_topic)
        rospy.loginfo("Publishing to "+self.topic_out)
        
        self.pt_pub = rospy.Publisher(self.topic_out,PointStamped)
        rospy.Subscriber(self.polaris_topic,PoseArray,self.callback)

    def start(self):
        rospy.spin()

    def callback(self,msg):
        try:
            if(self.tool_id<len(msg.poses)):
                pt_out = PointStamped()
                pt_out.header.frame_id = msg.header.frame_id
                pt_out.header.stamp = msg.header.stamp
                pt_out.point.x = msg.poses[self.tool_id].position.x
                pt_out.point.y = msg.poses[self.tool_id].position.y
                pt_out.point.z = msg.poses[self.tool_id].position.z
                if not (math.isnan(pt_out.point.x) or math.isnan(pt_out.point.y) or math.isnan(pt_out.point.z)):
                    self.pt_pub.publish(pt_out)
            else:
                rospy.logerr("tool_id ("+str(self.tool_id)+") is incorrect (msg.poses length :"+str(len(msg.poses))+")")
        except Exception,e:
            rospy.logerr(e)


def main(argv):
    P = PolarisListener()
    P.start()

if __name__ == '__main__':
    main(sys.argv)
    exit(0)


