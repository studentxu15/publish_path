#include "header.h"

class PublishPath : public ParamServer
{
public:
    ros::Subscriber sub_posestamp;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_odometry;

    ros::Publisher pub_path;

    vector<pose_time_vec> poses_vec;

    std::mutex poses_mutex;
    bool arrive_thre;

    PublishPath()
    {
        sub_posestamp = nh.subscribe<geometry_msgs::PoseStamped>(input_topic_posestamp, 100, &PublishPath::posestamp_callback, this);
        sub_pose = nh.subscribe<geometry_msgs::Pose>(input_topic_pose, 100, &PublishPath::pose_callback, this);
        sub_odometry = nh.subscribe<nav_msgs::Odometry>(input_topic_odometry, 100, &PublishPath::odometry_callback, this);

        pub_path = nh.advertise<nav_msgs::Path>(path_topic, 10);
        allocateMemory();
    }

    void allocateMemory()
    {
        arrive_thre = false;
    }

    double computer_distance(const vector<double>& this_pose, const vector<double>& last_pose)
    {
        double dis_x = this_pose[0] - last_pose[0];
        double dis_y = this_pose[1] - last_pose[1];
        double distance = std::sqrt(dis_x * dis_x + dis_y * dis_y);
        return distance;
    }

    void add_position(double x, double y, double z, double qx, double qy, double qz, double qw)
    {
        pose_time_vec ptv;
        ptv.pose.push_back(x);
        ptv.pose.push_back(y);
        ptv.pose.push_back(z);
        ptv.pose.push_back(qx);
        ptv.pose.push_back(qy);
        ptv.pose.push_back(qz);
        ptv.pose.push_back(qw);
        ptv.timestamp = ros::Time::now();

        int pos_size = poses_vec.size();
        if (pos_size > 1)
        {
            double this_distance = computer_distance(ptv.pose, poses_vec[pos_size - 1].pose);

            if (this_distance >= distance_threshold)
            {
                std::lock_guard<std::mutex> lock(poses_mutex);
                poses_vec.push_back(ptv);
                filtered_pose();
            }
        }
        else
        {
            std::lock_guard<std::mutex> lock(poses_mutex);
            poses_vec.push_back(ptv);
            filtered_pose();
        }
        
    }

    void posestamp_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        double pose_x = msg->pose.position.x;
        double pose_y = msg->pose.position.y;
        double pose_z = msg->pose.position.z;
        double pose_qx = msg->pose.orientation.x;
        double pose_qy = msg->pose.orientation.y;
        double pose_qz = msg->pose.orientation.z;
        double pose_qw = msg->pose.orientation.w;

        add_position(pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw);
    }

    void pose_callback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        double pose_x = msg->position.x;
        double pose_y = msg->position.y;
        double pose_z = msg->position.z;
        double pose_qx = msg->orientation.x;
        double pose_qy = msg->orientation.y;
        double pose_qz = msg->orientation.z;
        double pose_qw = msg->orientation.w;

        add_position(pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw);
    }

    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double pose_x = msg->pose.pose.position.x;
        double pose_y = msg->pose.pose.position.y;
        double pose_z = msg->pose.pose.position.z;
        double pose_qx = msg->pose.pose.orientation.x;
        double pose_qy = msg->pose.pose.orientation.y;
        double pose_qz = msg->pose.pose.orientation.z;
        double pose_qw = msg->pose.pose.orientation.w;

        add_position(pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw);

    }

    void filtered_pose()
    {
        if (num_limit == true && poses_vec.size() >  static_cast<std::vector<pose_time_vec>::size_type>(num_max_size))
        {
            if (!arrive_thre)
            {
                ROS_INFO("\033[1;32m----> The poses container reaches the threshold: %d, it will start to be deleted.\033[0m", num_max_size);
                arrive_thre = true;
            }
            
            poses_vec.erase(poses_vec.begin());
        }

        if (time_limit)
        {
            ros::Time current_time = ros::Time::now();
            while (!poses_vec.empty() && (current_time - poses_vec.front().timestamp).toSec() > time_max_limit) {
                poses_vec.erase(poses_vec.begin());
            }

        }
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