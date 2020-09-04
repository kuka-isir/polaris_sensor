// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h> // !!!
#include <sensor_msgs/PointCloud.h>
#include <serial/serial.h>
#include <polaris_sensor/polaris_sensor.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <fstream>
#include <std_msgs/Float32.h>
#include <ctime>
#include <iostream>

#include <tf/transform_broadcaster.h>  // !!! add the tf broadcaster.h
#include <tf/transform_listener.h> // !!! add the tf listener.h

bool fexists(const std::string& filename) {
  std::ifstream ifile(filename.c_str());
  return (bool)ifile;
}

using namespace boost;
using namespace std;
using namespace polaris;


bool nexists(const std::string& r)
{
    if(!fexists(r))
    {
        ROS_WARN("Rom %s doest not exists, skipping.",r.c_str());
        return true;
    }
    return false;
}

int main(int argc, char **argv)
{
    // Usage : rosrun polaris_sensor polaris_sensor _roms:="/home/T0.rom,/home/T1.rom" _port:=/dev/ttyUSB0
    ros::Time::init();
    ros::init(argc, argv, "polaris_sensor");
    ros::NodeHandle nh("~");

    ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("targets", 1);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("targets_cloud", 1);
    ros::Publisher dt_pub = nh.advertise<std_msgs::Float32>("dt", 1);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("targets_in_base", 1);

    tf::TransformBroadcaster broadcaster;  // !!! add the handle of broadcaster
    tf::TransformListener listener; // !!! add the handle of listener

    std::string port("/dev/ttyUSB0");
    if(!nh.getParam("port",port))
        ROS_WARN("Using default port: %s",port.c_str());
    else
        ROS_INFO("Using port: %s",port.c_str());

    std::string camera("polaris");
    if(!nh.getParam("camera",camera))
        ROS_WARN("Using default camera name: %s",camera.c_str());
    else
        ROS_INFO("Using camera name: %s",camera.c_str());

    std::vector<std::string> roms;
    std::string tmp;
    if(!nh.getParam ( "roms", tmp)){
        ROS_FATAL("No rom provided, exiting.");
        return -1;
    }
    boost::erase_all(tmp, " ");
    boost::split ( roms, tmp, boost::is_any_of(","));

    roms.erase(std::remove_if(roms.begin(),roms.end(),nexists),
                   roms.end());
   if(roms.size() == 0)
   {
       ROS_FATAL("No roms could be loaded, exiting.");
       return -2;
   }
    int n = roms.size();
    Polaris polaris(port,roms);

    geometry_msgs::PoseArray targets_pose;
    sensor_msgs::PointCloud targets_cloud;
    geometry_msgs::Pose target_base_pose; // !!! add the relative pose

    targets_cloud.header.frame_id = "/"+camera+"_link";
    targets_pose.header.frame_id = "/"+camera+"_link";

    ros::Rate loop_rate(100);
    int count = 0;
    ROS_INFO("Starting Polaris tracker loop");
    for(int i=0;i<n;++i){
      targets_pose.poses.push_back(geometry_msgs::Pose());
      targets_cloud.points.push_back(geometry_msgs::Point32());
    }

    geometry_msgs::Pose pose;
    geometry_msgs::Point32 pt;

    std_msgs::Float32 dt;
    std::map<int,TransformationDataTX> targets;
    float r_qx,r_qy,r_qz,r_qw,r_x,r_y,r_z; // !!! define the neccessary variable
    while (ros::ok())
    {
        /* Start TX */
        std::string status;


	ros::Time start = ros::Time::now();

	polaris.readDataTX(status,targets);

	ros::Time end = ros::Time::now();
	ros::Duration duration = (end - start);

	dt.data = duration.nsec/1000000.;

        dt_pub.publish(dt);

        std::map<int,TransformationDataTX>::iterator it = targets.begin();

        /* Start BX
        uint16_t status;
        std::map<int,TransformationDataBX> targets;
        polaris.readDataBX(status,targets);

        std::map<int,TransformationDataBX>::iterator it = targets.begin();*/

        broadcaster.sendTransform(
          tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(), "world", "ndi_base_link"));  // !!! broadcast tf frame of ndi base link
	unsigned int i=0;
        for(it = targets.begin();it!=targets.end();++it)
        {
            pose.position.x = it->second.tx;
            pose.position.y = it->second.ty;
            pose.position.z = it->second.tz;
            pose.orientation.x = it->second.qx;
            pose.orientation.y = it->second.qy;
            pose.orientation.z = it->second.qz;
            pose.orientation.w = it->second.q0;
            targets_pose.poses[i] = pose;

            pt.x = it->second.tx;
            pt.y = it->second.ty;
            pt.z = it->second.tz;
            targets_cloud.points[i] = pt;
	    i++;
        }
	if (isnan(targets_pose.poses[0].position.x)){
	  ROS_INFO("RoM0 is not in the range");}
	else if (isnan(targets_pose.poses[1].position.x)){
	  ROS_INFO("RoM1 is not in the range");}
        targets_cloud.header.stamp = ros::Time::now();
        targets_pose.header.stamp = ros::Time::now();
        cloud_pub.publish(targets_cloud);
        pose_array_pub.publish(targets_pose);
        broadcaster.sendTransform(
          tf::StampedTransform(tf::Transform(tf::Quaternion(targets_pose.poses[0].orientation.x, targets_pose.poses[0].orientation.y, targets_pose.poses[0].orientation.z, targets_pose.poses[0].orientation.w), tf::Vector3(targets_pose.poses[0].position.x, targets_pose.poses[0].position.y, targets_pose.poses[0].position.z)), ros::Time::now(), "ndi_base_link", "ndi_marker_base")); // !!! broadcast the tf frame of marker_base

        broadcaster.sendTransform(
          tf::StampedTransform(tf::Transform(tf::Quaternion(targets_pose.poses[1].orientation.x, targets_pose.poses[1].orientation.y, targets_pose.poses[1].orientation.z, targets_pose.poses[1].orientation.w), tf::Vector3(targets_pose.poses[1].position.x, targets_pose.poses[1].position.y, targets_pose.poses[1].position.z)), ros::Time::now(), "ndi_base_link", "ndi_marker_target")); // !!! broadcast the tf frame of marker_target

	//!!! publish the relative transformance between the base and target
	tf::StampedTransform transform; // !!!
        //!!!get the relation between ndi_marker_base and ndi_marker_target
        try{
          listener.waitForTransform("ndi_marker_target", "ndi_marker_base", ros::Time(0), ros::Duration(3.0));
          listener.lookupTransform("ndi_marker_target", "ndi_marker_base", ros::Time(0), transform);
           }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
	//!!! gettransform
        r_x=transform.getOrigin().x();
        r_y=transform.getOrigin().y();
        r_z=transform.getOrigin().z();
        //!!! getRotation
        r_qx=transform.getRotation()[0];
        r_qy=transform.getRotation()[1];
        r_qz=transform.getRotation()[2];
        r_qw=transform.getRotation()[3];
	// !!! add to the topic message
	target_base_pose.position.x = r_x;
	target_base_pose.position.y = r_y;
	target_base_pose.position.z = r_z;
	target_base_pose.orientation.x = r_qx;
	target_base_pose.orientation.y = r_qy;
	target_base_pose.orientation.z = r_qz;
	target_base_pose.orientation.w = r_qw;
	// !!! publish the topic "target_in_base"
	pose_pub.publish(target_base_pose);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
