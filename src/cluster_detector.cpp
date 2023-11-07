#include "cluster_detector/cluster_detector.hpp"

using std::placeholders::_1;

ClusterDetector::ClusterDetector() : Node("cluster_detector"){
    this->scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",1000,std::bind(&ClusterDetector::scan_callback, this, _1));
    this->obstacle_pub = this->create_publisher<cdf_msgs::msg::Obstacles>("raw_obstacles",1000);
    this->marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers",1000);
}

ClusterDetector::~ClusterDetector(){

}

void ClusterDetector::scan_callback(const sensor_msgs::msg::LaserScan msg){
    // If there isn't enough point, throw a warning
    if ((msg.ranges).size() != NB_POINT_SCAN ){
    	RCLCPP_WARN(this->get_logger(), "Not enough point in scan : %i out of %i",static_cast<int>((msg.ranges).size()), NB_POINT_SCAN);
    	return;
    } 
    
    float val_list[2*NB_MEDIAN+1];
    int i,add, access_index, buffer_index;
    float buffer_val;
    for (i = 0; i<NB_POINT_SCAN; i++){this->points[i].cluster_id=-1;}
    
    // Apply median filter to the scan
    for (i = 0; i < NB_POINT_SCAN; i++){
        // Place val in a list
        for (add = -NB_MEDIAN; add <= NB_MEDIAN; add++){
            access_index = i + add;
	    if (access_index < 0){access_index+=NB_POINT_SCAN;}
            else if (access_index >= NB_POINT_SCAN){access_index-=NB_POINT_SCAN;}
	    val_list[NB_MEDIAN+add] = (msg.ranges[access_index] < 10) ? msg.ranges[access_index] : 30;
	}

        // find median
        buffer_val = INT_MAX;
        for (int j = 0; j<NB_MEDIAN; j++){
            for (int k = 0; k<2*NB_MEDIAN+1-j; k++){
                if (val_list[k]<buffer_val){
                    buffer_val = val_list[k];
                    buffer_index = k;
                }
            }
            val_list[buffer_index] = val_list[2*NB_MEDIAN-j];
        }

	this->points[i].r = buffer_val;
	this->points[i].angle = i;
    }

    clusterize();
    
    create_message();
    
    publish_marker();
    publish_obstacle();
}



void ClusterDetector::clusterize(){
    /*
     * Create cluster of point in the array.
     * if a point do not belong in any cluster
     * the cluster_id will be -1.
     * This fuction assign a cluster_id to all 
     * PolarPoint in class variable points
     * */

    this->nb_cluster = -1;
    bool in_cluster = false;
    
    // Verify if there is a cluster at the beginning of the scan and therefore
    // if we need to browse the scan backward from the end
    bool back_needed = false;
    if (this->points[0].r < 10){
        in_cluster = true;
        back_needed = true;
        this->nb_cluster++;
        this->points[0].cluster_id = this->nb_cluster;
    }

    int i;
    for (i = 1; i < NB_POINT_SCAN; i++){
        if (in_cluster){
            if (std::abs(this->points[i-1].r-this->points[i].r) < DIST_CLUSTER){
                this->points[i].cluster_id = this->nb_cluster;
            }
            else{
                in_cluster = false;
            }
        }
        else{
            if (this->points[i].r < 10){
                in_cluster = true;
                this->nb_cluster++;
                this->points[i].cluster_id = this->nb_cluster;
            }
        }
    }
    
    // If there is cluster at the end and at the beginning joint the points
    if (back_needed && (this->points[NB_POINT_SCAN - 1].cluster_id == this->nb_cluster)){
        i = 1;
        while (this->points[NB_POINT_SCAN - i].cluster_id == this->nb_cluster){
            this->points[NB_POINT_SCAN - i].cluster_id = 0;
            i++;
        }
    }
}

void ClusterDetector::create_message(){
    /*
     * Store the calculated clusters in a message
     * of type cdf_msgs::msg::Obstacles.
     * */

    std::vector<std::vector<PolarPoint>> buffer_cluster(this->nb_cluster+1);
    
    int i,j,size;

    // Sort every point by its cluster_id
    for (i = 0; i<NB_POINT_SCAN; i++){
        if (this->points[i].cluster_id != -1){
            (buffer_cluster.at(this->points[i].cluster_id)).push_back(this->points[i]);
        }
    }
    
    float radius;
    float r, theta;

    // Solve the problem of a first cluster shared between the end and the beginning of the scan 
    if (this->nb_cluster>=0 && this->points[NB_POINT_SCAN-1].cluster_id == 0){
        i = buffer_cluster.front().size() - 1;
        // We transform angle near 2pi in negative angle
        while (buffer_cluster.front().at(i).angle > NB_POINT_SCAN) {
            buffer_cluster.front().at(i).angle -= NB_POINT_SCAN;
            i--;
        }
    }

    for (i = 0; i<this->nb_cluster; i++){
        r = 0;
        size = buffer_cluster[i].size();
        theta = 0;
        // Get the furthest point and consider it to be the center of the circle
        for (j = 0; j<size; j++){
            r = std::max(r,buffer_cluster.at(i).at(j).r);
            
            theta += buffer_cluster.at(i).at(j).angle;
        }
        // The angle to the center is the mean of all measured angles
        theta *= 2 * PI/(NB_POINT_SCAN * size);
        radius = 2 * sin(2 * PI *static_cast<double>(size)/(2*NB_POINT_SCAN)) * r;
        
        // Place the measured parameters in the message
        cdf_msgs::msg::CircleObstacle tmp_circle;
        tmp_circle.radius = radius;
        tmp_circle.center.x = r*cos(theta);
        tmp_circle.center.y = r*sin(theta);
            
        this->buffer_message.circles.push_back(tmp_circle);
    }
}

void ClusterDetector::publish_obstacle(){
    this->obstacle_pub->publish(this->buffer_message);
    buffer_message.circles.clear();
}

void ClusterDetector::publish_marker(){
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::MarkerArray flush;
    visualization_msgs::msg::Marker tmp;
    int i, prev_size;
    prev_size = this->nb_marker;

    this->nb_marker = this->buffer_message.circles.size();

    for (i = 0; i<this->nb_marker; i++){
        tmp.header.frame_id = "laser";
        tmp.action = visualization_msgs::msg::Marker::ADD;
        tmp.type = visualization_msgs::msg::Marker::SPHERE;
        tmp.id = i;
        tmp.scale.x = 0.1;
        tmp.scale.y = 0.1;
        tmp.scale.z = 0.1;
        tmp.color.a = 1;
        tmp.color.r = 0;
        tmp.color.g = 1;
        tmp.color.b = 0;
        tmp.pose.position.x = this->buffer_message.circles.at(i).center.x;
        tmp.pose.position.y = this->buffer_message.circles.at(i).center.y;
        tmp.pose.position.z = this->buffer_message.circles.at(i).center.z;
        markers.markers.push_back(tmp);
    }
    for (i = this->nb_marker; i<prev_size; i++){
        tmp.id = i;
        tmp.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(tmp);
    }

    this->marker_pub->publish(markers);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClusterDetector>());
    rclcpp::shutdown();
    return 0;
}
