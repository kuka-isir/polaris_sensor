# -*- coding: utf-8 -*-
"""
Created on Wed Dec 10 10:45:01 2014

@author: kuka-demo
"""

import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32
import numpy as np


class listener:
    def __init__(self):
        rospy.init_node('listener')
        
        rospy.Subscriber("/polaris_sensor/targets",PoseArray, self.callback)
        self.norm_pub = rospy.Publisher("/polaris_sensor/norm_pt",Float32)
        rospy.spin()
        
    def callback(self,data):
        x0 = data.poses[0].position.x
        y0 = data.poses[0].position.y
        z0 = data.poses[0].position.z
        x1 = data.poses[1].position.x
        y1 = data.poses[1].position.y
        z1 = data.poses[1].position.z

        distx = x1-x0
        disty = y1-y0
        distz = z1-z0   
        vect = np.array([distx,disty,distz])
        norm = np.linalg.norm(vect)
        
        f_out = Float32()
        f_out.data = norm
        
        rospy.loginfo(str(norm))
        self.norm_pub.publish(f_out)
    
if __name__=='__main__':
    L = listener()
        