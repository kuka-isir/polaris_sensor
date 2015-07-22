// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <serial/serial.h>
#include <polaris_sensor/polaris_sensor.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>

using namespace boost;
using namespace std;
using namespace polaris;



int main(int argc, char **argv)
{
    // Usage : rosrun polaris_sensor polaris_sensor _roms:="/home/T0.rom,/home/T1.rom" _port:=/dev/ttyUSB0
    ros::Time::init();
    ros::init(argc, argv, "polaris_sensor");
    ros::NodeHandle nh("~");

    ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("targets", 100);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("targets_cloud", 100);

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

    roms.erase(std::remove_if(roms.begin(),roms.end(),
                                [](std::string r)
                                {
                                    if(!boost::filesystem::exists(r))
                                    {
                                        ROS_WARN("Rom %s doest not exists, skipping.",r.c_str());
                                        return true;
                                    }
                                    return false;
                                }),
                   roms.end());
   if(roms.size() == 0)
   {
       ROS_FATAL("No roms could be loaded, exiting.");
       return -2;
   }

    Polaris polaris(port,roms);

    geometry_msgs::PoseArray targets_pose;
    sensor_msgs::PointCloud targets_cloud;

    targets_cloud.header.frame_id = "/"+camera+"_link";
    targets_pose.header.frame_id = "/"+camera+"_link";

    ros::Rate loop_rate(100);
    int count = 0;
    ROS_INFO("Starting Polaris tracker loop");
    while (ros::ok())
    {
        targets_pose.poses.clear();
        targets_cloud.points.clear();
        /* Start TX */
        std::string status;
        std::map<int,TransformationDataTX> targets;
        polaris.readDataTX(status,targets);

        std::map<int,TransformationDataTX>::iterator it = targets.begin();

        /* Start BX
        uint16_t status;
        std::map<int,TransformationDataBX> targets;
        polaris.readDataBX(status,targets);

        std::map<int,TransformationDataBX>::iterator it = targets.begin();*/
        for(;it!=targets.end();++it)
        {
            geometry_msgs::Pose pose;
            pose.position.x = it->second.tx;
            pose.position.y = it->second.ty;
            pose.position.z = it->second.tz;
            pose.orientation.x = it->second.qx;
            pose.orientation.y = it->second.qy;
            pose.orientation.z = it->second.qz;
            pose.orientation.w = it->second.q0;
            targets_pose.poses.push_back(pose);

            geometry_msgs::Point32 pt;
            pt.x = it->second.tx;
            pt.y = it->second.ty;
            pt.z = it->second.tz;
            targets_cloud.points.push_back(pt);
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
