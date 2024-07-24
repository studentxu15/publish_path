#pragma once
#ifndef HEADER_H
#define HEADER_H

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <thread>
#include <vector>
#include <nav_msgs/Path.h>
#include "publish_path/CustomMsg.h"

using namespace std;

struct pose_time_vec {
    vector<double> pose;
    ros::Time timestamp;
};

class ParamServer
{
public:
	ros::NodeHandle nh;
	ros::NodeHandle private_nh;

	std::string PROJECT_NAME;
    std::string input_topic;
    std::string world_frame;
    std::string path_topic;
    double pub_frequency;

    bool num_limit;
    int num_max_size;

    bool time_limit;
    double time_max_limit;


	ParamServer()
		: private_nh("~")
	{
		nh.param<std::string>("/PROJECT_NAME", PROJECT_NAME, "publish_path");
        private_nh.param<std::string>("input_topic", input_topic, "/pose");
        private_nh.param<std::string>("world_frame", world_frame, "map");
        private_nh.param<std::string>("path_topic", path_topic, "/path");
        private_nh.param<double>("pub_frequency", pub_frequency, 10.0);

        private_nh.param<bool>("num_limit", num_limit, true);
        private_nh.param<int>("num_max_size", num_max_size, 20000);

        private_nh.param<bool>("time_limit", time_limit, false);
        private_nh.param<double>("time_max_limit", time_max_limit, 100.0);
	}
};
#endif // HEADER_H