#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <deque>

class TrajectoryPublisher
{
    public:
        ros::Publisher marker_pub_;

        TrajectoryPublisher()
        {
            ros::NodeHandle nh_;
            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualisation_marker_array", 1000);
        }

        void readTrajectoryFile(const std::string& filename)
        {   
            std::ifstream file(filename); 
            if (!file.is_open())
            {
                ROS_ERROR_STREAM("Failed to open file: " << filename);
                return;
            }

            std::string line;
            std::deque<geometry_msgs::PoseStamped> trajectory;
            std::getline(file, line);

            while (std::getline(file, line)) 
            {
                std::stringstream ss(line);
                std::string item;
                std::vector<double> elements;
                while (std::getline(ss, item, ',')) 
                {
                    elements.push_back(std::stod(item));
                }
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = elements[0];
                pose.pose.position.y = elements[1];
                pose.pose.position.z = elements[2];
                pose.pose.orientation.x = elements[3];
                pose.pose.orientation.y = elements[4];
                pose.pose.orientation.z = elements[5];
                pose.pose.orientation.w = elements[6];
                trajectory.push_back(pose);
            }

            file.close();
            
            while(ros::ok())
            {
                publishTrajectory(trajectory);
                ros::Duration(1).sleep();
            }
            
        }
        void publishTrajectory(const std::deque<geometry_msgs::PoseStamped>& trajectory)
        {   
            ROS_INFO("Publishing trajectory with %zu poses", trajectory.size());
            visualization_msgs::MarkerArray markers;
            int id = 0;
            for (const auto& pose : trajectory)
            {   
                visualization_msgs::Marker marker;
                marker.header.frame_id = "odom";
                marker.header.stamp = ros::Time::now();        
                marker.ns = "trajectory_markers";
                marker.id = id++;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose = pose.pose;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.r = 1.0;
                marker.color.a = 1.0;
                markers.markers.push_back(marker);
            ROS_INFO("Added marker %d at (%f, %f, %f)", marker.id, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            }
            ROS_INFO("published");
            marker_pub_.publish(markers);
        }
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_publisher");

    if (argc < 2)
    {
        ROS_ERROR("Usage: rosrun trajectory_manager trajectory_publisher <trajectory_file>");
        return 1;
    }

    TrajectoryPublisher publisher;
    publisher.readTrajectoryFile(argv[1]);

    // ros::spin();
    return 0;
}