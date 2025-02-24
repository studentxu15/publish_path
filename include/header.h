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
#include <dynamic_reconfigure/server.h>
#include "publish_path/ReconfigConfig.h"
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

    double distance_threshold;

    std::string input_type;

    std::string input_topic_posestamp;
    std::string input_topic_pose;
    std::string input_topic_odometry;


	ParamServer()
		: private_nh("~")
	{
		nh.param<std::string>("/PROJECT_NAME", PROJECT_NAME, "publish_path");
        nh.param<double>(PROJECT_NAME + "/distance_threshold", distance_threshold, 0.01);
        private_nh.param<std::string>("input_topic", input_topic, "/pose00");
        private_nh.param<std::string>("world_frame", world_frame, "map");
        private_nh.param<std::string>("path_topic", path_topic, "/path");
        private_nh.param<std::string>("input_type", input_type, "pp");
        private_nh.param<double>("pub_frequency", pub_frequency, 10.0);

        private_nh.param<bool>("num_limit", num_limit, true);
        private_nh.param<int>("num_max_size", num_max_size, 20000);

        private_nh.param<bool>("time_limit", time_limit, false);
        private_nh.param<double>("time_max_limit", time_max_limit, 100.0);

        initial_topic();
	}

    void initial_topic()
    {
        input_topic_posestamp = "/topic00";
        input_topic_pose = "/topic01";
        input_topic_odometry = "/topic02";

        if (input_type == "posestamp")
        {
            input_topic_posestamp = input_topic;
        }
        else if (input_type == "pose")
        {
            input_topic_pose = input_topic;
        }
        else if (input_type == "odometry")
        {
            input_topic_odometry = input_topic;
        }
        else
        {
            ROS_ERROR("input_type INPUT ERROR, Please enter the corrent 'input_type' <'posestamp', 'pose' or 'odometry'>");
        }
    }
};
#endif // HEADER_H