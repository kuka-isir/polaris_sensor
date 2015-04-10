#!/usr/bin/env python
'''
Created on Nov 27, 2014

@author: Antoine Hoarau <hoarau.robotics@gmail.com>
'''

import rospy
import tf
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from threading import Thread
import numpy as np 
import sys
import argparse
import textwrap
from tf.transformations import quaternion_from_matrix
import message_filters
from message_filters import TimeSynchronizer
import itertools
import time
from threading import Lock,Event

def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".
    """
    valid = {"yes": True, "y": True, "ye": True,
             "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "
                             "(or 'y' or 'n').\n")

class ApproximateTimeSynchronizer(TimeSynchronizer):

    """
    Approximately synchronizes messages by their timestamps.
    :class:`ApproximateTimeSynchronizer` synchronizes incoming message filters by the
    timestamps contained in their messages' headers. The API is the same as TimeSynchronizer
    except for an extra `slop` parameter in the constructor that defines the delay (in seconds)
    with which messages can be synchronized
    """

    def __init__(self, fs, queue_size, slop):
        TimeSynchronizer.__init__(self, fs, queue_size)
        self.slop = rospy.Duration.from_sec(slop)

    def add(self, msg, my_queue):
        self.lock.acquire()
        my_queue[msg.header.stamp] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        for vv in itertools.product(*[list(q.keys()) for q in self.queues]):
            qt = list(zip(self.queues, vv))
            if ( ((max(vv) - min(vv)) < self.slop) and
                (len([1 for q,t in qt if t not in q]) == 0) ):
                msgs = [q[t] for q,t in qt]
                self.signalMessage(*msgs)
                for q,t in qt:
                    del q[t]
        self.lock.release()
        
def rigid_transform_3D(A, B):
    assert len(A) == len(B)
    N = A.shape[0]; # total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))
    # dot is matrix multiplication for array
    H = np.transpose(AA) * BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T * U.T
    # special reflection case
    if np.linalg.det(R) < 0:
        Vt[2,:] *= -1
        R = Vt.T * U.T
    t = -R*centroid_A.T + centroid_B.T
    return R, t

class TfBroadcasterThread(Thread):
    def __init__(self,child_frame,parent_frame,tf_br=None):
        Thread.__init__(self)
        rospy.loginfo("Initializing tf broadcaster with child frame "+child_frame+" and parent frame "+parent_frame)
        if tf_br is None:
            self.tf_br = tf.TransformBroadcaster()
        else:
            self.tf_br = tf_br
        self.translation = None
        self.quaternion = None
        self.child_frame = child_frame
        self.parent_frame = parent_frame
        self.has_transformation=False
        self.stamp = None
        self.stop = Event()
        self.lock=Lock()
            
    def set_transformation(self,translation,quaternion,stamp=None):
        self.lock.acquire()
        self.translation = translation
        self.quaternion = quaternion
        if not stamp:
            self.stamp = rospy.Time.now()
        else:
            self.stamp = stamp
        if not self.has_transformation:
            self.has_transformation =True
        self.lock.release()
        

    def run(self):
        r = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            try:
                if self.has_transformation:
                    if self.lock.acquire(False):
                        self.tf_br.sendTransform(self.translation ,self.quaternion , rospy.Time.now(), self.child_frame,self.parent_frame)
                        self.lock.release()
            except Exception,e:
                rospy.logerr('TfBroadcasterThread:'+str(e))
                #return
            r.sleep()
             
class Estimator:
    def __init__(self,topic_a,topic_b,child_frame,parent_frame,transform_name="calib_kinect",dist_min_between_pts=0.03,n_pts_to_start_calib=3,slope=0.1,queue_size=10,output_file=None,enable_exact_timestamps=False):
        rospy.init_node("compute_transformation_6d")
        self.rate = rospy.Rate(100)
        self.topic_a = topic_a
        self.topic_b = topic_b
        self.child_frame = child_frame
        self.parent_frame = parent_frame
        self.pts_a = []
        self.pta_stamp = None
        self.ptb_stamp = None
        self.pts_b = []
        self.start_tf_broadcaster()
        self.static_transform = []
        self.init_subscribers(enable_exact_timestamps=enable_exact_timestamps,queue_size=100,slope=slope)
        self.transform_name = transform_name
        self.MIN_NUM_CALIB = n_pts_to_start_calib
        self.min_d = dist_min_between_pts
        self.lock=Lock()
        self.do_calib = Event()
        self.cloud_a_pub = rospy.Publisher("/pointsA",PointCloud)
        self.cloud_b_pub = rospy.Publisher("/pointsB",PointCloud)
        self.output_file_path = output_file
        self.static_transform = None
        self.translation = None
        self.quaternion = None
        self.open_last_calibration(output_file)
        
    def init_subscribers(self,enable_exact_timestamps=False,queue_size=1,slope=1.0):
        rospy.loginfo("Initializing subscribers")
        sub=[]
        sub.append(message_filters.Subscriber(self.topic_a, PointStamped))
        sub.append(message_filters.Subscriber(self.topic_b, PointStamped))
        rospy.loginfo("Listening to "+self.topic_a+" and "+self.topic_b)
        if enable_exact_timestamps:
            ts = TimeSynchronizer(sub, queue_size)
        else:
            ts = ApproximateTimeSynchronizer(sub, queue_size,slope)
        rospy.loginfo("Queue size is "+str(queue_size)+", slope is "+str(slope))
        #ts.registerCallback(self.callback)        
        #ts = message_filters.TimeSynchronizer(sub, 20)
        ts.registerCallback(self.callback)
        
    def start_tf_broadcaster(self):        
        rospy.loginfo("Initializing the Tf broadcaster")
        self.tf = tf.TransformListener()
        self.tf_thread = TfBroadcasterThread(self.parent_frame,self.child_frame)
        self.tf_thread.start()
    
    def publish_pointcloud(self,cloud,frame_out,cloud_publisher,stamp=None):
        cloud_out = PointCloud()
        cloud_out.header.frame_id = frame_out
        if not stamp:
            cloud_out.header.stamp = rospy.Time.now()
        else:
            cloud_out.header.stamp = stamp
        for p in cloud:
            p_out = Point32()
            p_out.x = p.item(0)
            p_out.y = p.item(1)
            p_out.z = p.item(2)
            cloud_out.points.append(p_out)
        cloud_publisher.publish(cloud_out)
              
    def is_point_far_enough(self,p_in,list_of_pt,dmin):
        for p in list_of_pt:
            p1 = np.array(p)
            p2 = np.array(p_in)
            vd = p1-p2
            d = np.linalg.norm(vd)
            if d<dmin:
                return False
        return True
            
    def callback(self,*msg):
        add_new_pt = True
        pta = [msg[0].point.x ,msg[0].point.y ,msg[0].point.z]
        ptb = [msg[1].point.x ,msg[1].point.y ,msg[1].point.z]
        
        #self.lock.acquire()
        #if(len(self.pts_a)>=self.MIN_NUM_CALIB):
        add_new_pt = self.is_point_far_enough(pta,self.pts_a,self.min_d)
        #self.lock.release()
        #if len(self.pts_a)>0 and np.linalg.norm([self.pts_a[-1],pta]) < self.min_d:
        #    rospy.logwarn("Point to close, not adding ",pta," d=",np.linalg.norm([self.pts_a[-1],pta]),"dmin=",self.min_d)
        #    add_new_pt = False
        if add_new_pt:
            diff = msg[1].header.stamp - msg[0].header.stamp
            rospy.loginfo("dt between msgs : "+str(diff/1e6)+"ms")
            #rospy.loginfo("DIFF : "+str(diff/1e3)+"us")            
            #rospy.loginfo("DIFF : "+str(diff)+"ns")
            #self.lock.acquire()
            self.pta_stamp = msg[0].header.stamp
            self.ptb_stamp = msg[1].header.stamp
            
            self.pts_a.append(pta)
            self.pts_b.append(ptb)
            #self.lock.release()
            
            #self.do_calib.set()
            
            #self.lock.acquire()
            A = np.matrix(self.pts_a)
            B = np.matrix(self.pts_b)
            #self.lock.release()

            if len(self.pts_a):
                self.publish_pointcloud(A,self.child_frame,self.cloud_a_pub,self.pta_stamp)
            if len(self.pts_b):
                self.publish_pointcloud(B,self.parent_frame,self.cloud_b_pub,self.ptb_stamp)
            #print "Points A"
            #print A
            #print ""
            
            #print "Points B"
            #print B
            #print ""
            if len(self.pts_a)>=self.MIN_NUM_CALIB:
                ret_R, ret_t = rigid_transform_3D(B, A)
                new_col = ret_t.reshape(3, 1)
                tmp = np.append(ret_R, new_col, axis=1)
                aug=np.array([[0.0,0.0,0.0,1.0]])
                self.translation = np.squeeze(np.asarray(ret_t))
                T = np.append(tmp,aug,axis=0)
                self.quaternion = quaternion_from_matrix(T)
                #self.translation = ret_t
            
            try:
                self.tf_thread.set_transformation(self.translation,self.quaternion,self.ptb_stamp)
            except Exception,e:
                rospy.logerr(str(e))
                
            if self.translation is not None:
                #print 'type:',type(self.translation),self.translation
                self.static_transform = '<node pkg="tf" type="static_transform_publisher" name="'+self.transform_name+'" args="'\
                +' '.join(map(str, self.translation))+' '+' '.join(map(str, self.quaternion))+' '+self.child_frame+' '+self.parent_frame+' 100" />'
                #+str(self.translation[0])+' '+str(self.translation[1])+' '+str(self.translation[2])+' '+' '.join(map(str, self.quaternion))+' '+self.child_frame+' '+self.parent_frame+' 100" />'
            
            print ""
            print self.static_transform
            print ""
            #print "Translation - Rotation"
            #print self.translation,self.quaternion
		 


            #rospy.loginfo("Waiting for %d new points to start calibration."%(self.MIN_NUM_CALIB-len(self.pts_a)))
            #time.sleep(1.0)
            
        else:
            pass
            #rospy.logwarn("Point too close")
        #time.sleep(2.0)
        
    def start(self):
        try:
            while not rospy.is_shutdown():
                #self.do_calib.wait()
                self.rate.sleep()
        except Exception,e:
            print "Final exception : ",e

    def open_last_calibration(self,output_file_path):
        if not self.output_file_path:
            print 'Not openning last calib'
            return
        try:
            with open(self.output_file_path,'r') as f:
                s = f.read()
                i_beg = s.find('args="') + 6
                i_end = s.find('"',i_beg)
                sub = s[i_beg:i_end]
                l = sub.split(' ')
                if len(l) == 10:
                    self.parent_frame = l[-2]
                    self.child_frame = l[-3]
                    self.translation = np.array([float(l[0]),float(l[1]),float(l[2])],dtype=np.float64)
                    self.quaternion = np.array([float(l[3]),float(l[4]),float(l[5]),float(l[6])],dtype=np.float64)
                    print "trans = ",self.translation
                    print "quat = ",self.quaternion
                    print "parentf = ",self.parent_frame
                    print "childf = ",self.child_frame
                
        except Exception,e: print e
            
    def save_calibration(self):
        if not self.static_transform or not self.output_file_path:
            print 'Not saving files'
            return
        if query_yes_no("Do you want to save "+str(self.output_file_path)):
            print "Saving file ",self.output_file_path
            try:
                with open(self.output_file_path,'r') as f:
                    with open(self.output_file_path+'.bak','w') as fbak:
                        print self.output_file_path,' already exists, creating backup file.'
                        fbak.write(f.read())
            except Exception,e: print e
            with open(self.output_file_path,'w') as f:
                print self.static_transform
                f.write("""
<launch>
   """+self.static_transform+
"""
</launch>
""")
            print "File saved."
        else:
            print "Not saving calibration."
        
        
            
        
def main(argv):
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,description=textwrap.dedent(""" Estimates the best 6D transformation between two set of 3D points"""),epilog='Maintainer: Antoine Hoarau <hoarau.robotics AT gmail DOT com>')
    parser.add_argument('topic_a', type=str,help='First topic to listen (geometry_msgs::PointStamped)',default='/tracker/ball_position')
    parser.add_argument('topic_b', type=str,help='Second topic to listen (geometry_msgs::PointStamped)  ',default='/kuka/tooltip_position')    
    parser.add_argument('child_frame', type=str,help='Child Frame (should match first topic)',default='/camera_depth_optical_frame')
    parser.add_argument('parent_frame', type=str,help='Second Frame (should match second topic)',default='/base_link')
    parser.add_argument('--name', type=str,help='Transformation name (for roslaunch)',default='calib_kinect')
    parser.add_argument('-d','--dmin_between_pts', type=float,help='Distance min to get a new point',default=0.01)
    parser.add_argument('-m','--min_pts_to_start', type=int,help='Min number of points before we start calibrating',default=3)
    parser.add_argument('-s','--slope', type=float,help='Slope time in seconds for approximate time synchronizer',default=0.02)
    parser.add_argument('-q','--queue_size', type=int,help='Queue size for messages',default=100)
    parser.add_argument('-o','--output_file', type=str,help='The output file for the calibration (default none, i.e not saving)',default=None)
    parser.add_argument('-e','--enable_exact_timestamps', type=bool,help='Enable exact timestamp for topic A and B',default=False)
    args,_ = parser.parse_known_args()
    E = Estimator(args.topic_a,args.topic_b,args.child_frame,args.parent_frame,args.name,args.dmin_between_pts,args.min_pts_to_start,args.slope,args.queue_size,args.output_file,args.enable_exact_timestamps)
    E.start()
    E.save_calibration()
    
if __name__ == '__main__':
    main(sys.argv)
    exit(0)
    

