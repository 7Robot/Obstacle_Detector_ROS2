#ifndef CLUSTER_DETECTOR_HPP
#define CLUSTER_DETECTOR_HPP

#ifndef INT_MAX
#define INT_MAX 2147483647
#endif

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cdf_msgs/msg/obstacles.hpp"
#include "cdf_msgs/msg/circle_obstacle.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <cmath>
#include <vector>

#define NB_POINT_SCAN 1600
#define NB_MEDIAN 2
#define DIST_CLUSTER 0.1
#define PI 3.141592653589793

struct PolarPoint {
    float r;
    int angle;
    int cluster_id = -1;
};

class ClusterDetector : public rclcpp::Node{
public:
    ClusterDetector();
    ~ClusterDetector();
private:
    void scan_callback(const sensor_msgs::msg::LaserScan msg);
    void clusterize();
    void create_message();
    void publish_obstacle();
    
    int nb_cluster;

    PolarPoint points[NB_POINT_SCAN];
    cdf_msgs::msg::Obstacles buffer_message;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<cdf_msgs::msg::Obstacles>::SharedPtr obstacle_pub;

    // Variables linked to markers
    int nb_marker = 0;
    void publish_marker();
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
};

#endif
