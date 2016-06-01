// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
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

        targets_cloud.header.stamp = ros::Time::now();
        targets_pose.header.stamp = ros::Time::now();
        cloud_pub.publish(targets_cloud);
        pose_array_pub.publish(targets_pose);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
