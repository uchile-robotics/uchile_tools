#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>

class LidarCroppingNode {
public:
    LidarCroppingNode() : nh("~") {
        ROS_INFO("Getting camera fov info...");

        nh.param<double>("camera_fov_start_angle", camera_fov_start_angle, -45.0);
        nh.param<double>("camera_fov_end_angle", camera_fov_end_angle, 45.0);
        nh.param<std::string>("laser_topic", laser_topic, "/hokuyo_front_topic");

        lidar_full_sub = nh.subscribe("/scan", 10, &LidarCroppingNode::lidarFullCallback, this);
        lidar_cropped_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_cropped", 10);

        ROS_INFO("Setting lidar fov [nav] from %f to %f", camera_fov_start_angle, camera_fov_end_angle);
    }


    void lidarFullCallback(const sensor_msgs::LaserScan::ConstPtr& lidar_scan) {
        ROS_INFO_ONCE("Received lidar data.");

        sensor_msgs::LaserScan cropped_scan;
        cropped_scan.header = lidar_scan->header;
        cropped_scan.angle_min = degreesToRadians(camera_fov_start_angle);
        cropped_scan.angle_max = degreesToRadians(camera_fov_end_angle);
        cropped_scan.angle_increment = lidar_scan->angle_increment;
        cropped_scan.time_increment = lidar_scan->time_increment;
        cropped_scan.scan_time = lidar_scan->scan_time;
        cropped_scan.range_min = lidar_scan->range_min;
        cropped_scan.range_max = lidar_scan->range_max;

        int start_index = angleToIndex(camera_fov_start_angle, lidar_scan->angle_min, lidar_scan->angle_increment);
        int end_index = angleToIndex(camera_fov_end_angle, lidar_scan->angle_min, lidar_scan->angle_increment);

        // ROS_INFO("Cropping from index %d to %d", start_index, end_index);

        // Crop lidar ranges to match camera FOV
        for (int i = start_index; i <= end_index; i++) {
            cropped_scan.ranges.push_back(lidar_scan->ranges[i]);
        }

        // ROS_INFO("Cropped scan size: %zu", cropped_scan.ranges.size());

        lidar_cropped_pub.publish(cropped_scan);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_full_sub;
    ros::Publisher lidar_cropped_pub;
    double camera_fov_start_angle;
    double camera_fov_end_angle;
    std::string laser_topic;

    int angleToIndex(double angle_deg, double angle_min, double angle_increment) {
        double angle_rad = degreesToRadians(angle_deg);
        int id = static_cast<int>((angle_rad - angle_min) / angle_increment);
        return std::max(id, 0);
    }

    double degreesToRadians(double angle_deg) {
        return angle_deg * (M_PI / 180.0);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_cropping_node");
    LidarCroppingNode node;
    ros::spin();
    return 0;
}
