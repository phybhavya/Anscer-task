#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <anscer_task/saveTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <deque>
#include <fstream>

class TrajectoryCollector {
public:
    TrajectoryCollector() : max_duration_(ros::Duration(50.0)) {
        pose_sub_ = nh_.subscribe("/odom", 10, &TrajectoryCollector::odomCallback, this);
        trajectory_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 10);
        save_trajectory_service_ = nh_.advertiseService("saveTrajectory", &TrajectoryCollector::saveTrajectoryCallback, this);
    }
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher trajectory_pub_;
    ros::ServiceServer save_trajectory_service_;
    std::deque<geometry_msgs::PoseStamped> trajectory_;
    ros::Duration max_duration_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        visualization_msgs::MarkerArray trajectory_marker_array;  
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        trajectory_.push_back(pose);
        while (!trajectory_.empty() && pose.header.stamp - trajectory_.front().header.stamp > max_duration_) {
            trajectory_.pop_front();
        }
        visualization_msgs::MarkerArray markers;
        markers.markers.resize(trajectory_.size());
        int i = 0;
        for(const auto& pose : trajectory_) {
            visualization_msgs::Marker& marker = markers.markers[i++];
            marker.header.stamp = ros::Time::now();
            marker.header = pose.header;
            marker.ns = "trajectory_markers";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = pose.pose;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            trajectory_marker_array.markers.push_back(marker);
        }

        trajectory_pub_.publish(trajectory_marker_array);
    }

    bool saveTrajectoryCallback(anscer_task::saveTrajectory::Request& req,
                                anscer_task::saveTrajectory::Response& res)
    {
        std::ofstream file(req.filename.c_str());
        if (!file.is_open()) {
            res.success = false;
            res.message = "Failed to open file for writing";
            return true;
        }

        ros::Time start_time = ros::Time::now() - ros::Duration(req.duration);
        for (const auto& pose : trajectory_) {
            if (pose.header.stamp >= start_time) {
                file << pose.pose.position.x << ","
                    << pose.pose.position.y << ","
                    << pose.pose.position.z << ","
                    << pose.pose.orientation.x << ","
                    << pose.pose.orientation.y << ","
                    << pose.pose.orientation.z << ","
                    << pose.pose.orientation.w << std::endl;
            }
        }

        file.close();

        res.success = true;
        res.message = "Trajectory data saved successfully";
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_collector");
    TrajectoryCollector collector;
    ros::spin();
    return 0;
}