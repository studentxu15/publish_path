#include "header.h"

class PublishPath : public ParamServer
{
public:
    // ros::Subscriber sub_posestamp;
    // ros::Subscriber sub_pose;
    // ros::Subscriber sub_odometry;

    ros::Subscriber sub_custom;

    ros::Publisher pub_path;

    vector<pose_time_vec> poses_vec;

    std::mutex poses_mutex;

    PublishPath()
    {
        // sub_posestamp = nh.subscribe<geometry_msgs::PoseStamped>(input_topic, 100, &PublishPath::posestamp_callback, this);
        // sub_pose = nh.subscribe<geometry_msgs::Pose>(input_topic, 100, &PublishPath::pose_callback, this);
        // sub_odometry = nh.subscribe<nav_msgs::Odometry>(input_topic, 100, &PublishPath::odometry_callback, this);

        sub_custom = nh.subscribe<publish_path::CustomMsg>("input_topic", 100, &PublishPath::custom_callback, this);

        pub_path = nh.advertise<nav_msgs::Path>(path_topic, 10);
        allocateMemory();
    }

    void allocateMemory()
    {

    }

    void custom_callback(const publish_path::CustomMsg::ConstPtr& msg)
    {
        if (msg->pose_stamped.header.stamp != ros::Time(0))
        {
            posestamp_callback(boost::make_shared<geometry_msgs::PoseStamped>(msg->pose_stamped));
        }
        else if (msg->pose.position.x != 0 || msg->pose.position.y != 0 || msg->pose.position.z != 0)
        {
            pose_callback(boost::make_shared<geometry_msgs::Pose>(msg->pose));
        }
        else if (msg->odometry.header.stamp != ros::Time(0))
        {
            odometry_callback(boost::make_shared<nav_msgs::Odometry>(msg->odometry));
        }
    }

    void posestamp_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose_time_vec ptv;
        ptv.pose.push_back(msg->pose.position.x);
        ptv.pose.push_back(msg->pose.position.y);
        ptv.pose.push_back(msg->pose.position.z);
        ptv.pose.push_back(msg->pose.orientation.x);
        ptv.pose.push_back(msg->pose.orientation.y);
        ptv.pose.push_back(msg->pose.orientation.z);
        ptv.pose.push_back(msg->pose.orientation.w);
        ptv.timestamp = msg->header.stamp;

        std::lock_guard<std::mutex> lock(poses_mutex);
        poses_vec.push_back(ptv);
    }

    void pose_callback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        pose_time_vec ptv;
        ptv.pose.push_back(msg->position.x);
        ptv.pose.push_back(msg->position.y);
        ptv.pose.push_back(msg->position.z);
        ptv.pose.push_back(msg->orientation.x);
        ptv.pose.push_back(msg->orientation.y);
        ptv.pose.push_back(msg->orientation.z);
        ptv.pose.push_back(msg->orientation.w);
        ptv.timestamp = ros::Time::now();

        std::lock_guard<std::mutex> lock(poses_mutex);
        poses_vec.push_back(ptv);
    }

    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        pose_time_vec ptv;
        ptv.pose.push_back(msg->pose.pose.position.x);
        ptv.pose.push_back(msg->pose.pose.position.y);
        ptv.pose.push_back(msg->pose.pose.position.z);
        ptv.pose.push_back(msg->pose.pose.orientation.x);
        ptv.pose.push_back(msg->pose.pose.orientation.y);
        ptv.pose.push_back(msg->pose.pose.orientation.z);
        ptv.pose.push_back(msg->pose.pose.orientation.w);
        ptv.timestamp = msg->header.stamp;

        std::lock_guard<std::mutex> lock(poses_mutex);
        poses_vec.push_back(ptv);

    }

    void path_publish()
    {
        ros::Rate rate(pub_frequency);
        while (ros::ok())
        {
            nav_msgs::Path path_msg;
            path_msg.header.stamp = ros::Time::now();
            path_msg.header.frame_id = world_frame;
            {
                std::lock_guard<std::mutex> lock(poses_mutex);
                if (poses_vec.size() > 2)
                {
                    
                    for (const auto& ptv : poses_vec) {
                        geometry_msgs::PoseStamped pose_stamped;
                        pose_stamped.header.stamp = ptv.timestamp;
                        pose_stamped.header.frame_id = world_frame;
                        pose_stamped.pose.position.x = ptv.pose[0];
                        pose_stamped.pose.position.y = ptv.pose[1];
                        pose_stamped.pose.position.z = ptv.pose[2];
                        pose_stamped.pose.orientation.x = ptv.pose[3];
                        pose_stamped.pose.orientation.y = ptv.pose[4];
                        pose_stamped.pose.orientation.z = ptv.pose[5];
                        pose_stamped.pose.orientation.w = ptv.pose[6];

                        path_msg.poses.push_back(pose_stamped);
                    }

                    pub_path.publish(path_msg);
                }
            }
            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_path");

    PublishPath PP;

    ROS_INFO("\033[1;32m----> Publish Path Node START.\033[0M");

    std::thread PublishpathThread(&PublishPath::path_publish, &PP);

    ros::spin();

    PublishpathThread.join();

    return 0;
}