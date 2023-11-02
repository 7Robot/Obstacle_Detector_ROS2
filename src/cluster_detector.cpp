#include "cluster_detector/cluster_detector.hpp"

using std::placeholders::_1;

ClusterDetector::ClusterDetector() : Node("cluster_detector"){
    this->scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",1000,std::bind(&ClusterDetector::scan_callback, this, _1));
    this->obstacle_pub = this->create_publisher<cdf_msgs::msg::Obstacles>("raw_obstacles",1000);
}

ClusterDetector::~ClusterDetector(){

}

void ClusterDetector::scan_callback(const sensor_msgs::msg::LaserScan msg){
    // If there isn't enough point, throw a warning
    if ((msg.ranges).size() != NB_POINT_SCAN ){
    	RCLCPP_WARN(this->get_logger(), "Not enough point in scan : %i out of %i",static_cast<int>((msg.ranges).size()), NB_POINT_SCAN);
    	return;
    } 
    
    int val_list[2*NB_MEDIAN+1];
    int i,add, access_index, buffer_index;
    float buffer_val;
    for (i = 0; i<NB_POINT_SCAN; i++){this->points[i].cluster_id=-1;}
    
    // Apply median filter to the scan
    for (i = 0; i < NB_POINT_SCAN; i++){
        // Place val in a list
        for (add = -NB_MEDIAN; add <= NB_MEDIAN; add++){
            access_index = i + add;
	    if (access_index < 0){access_index+=NB_POINT_SCAN;}
	    if (access_index >= NB_POINT_SCAN){access_index-=NB_POINT_SCAN;}
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
    
    publish_obstacle();
}



void ClusterDetector::clusterize(){
    /*
     * Create cluster of point in the array.
     * if a point do not belong in any cluster
     * the cluster_id will be -1
     * */

    this->nb_cluster = -1;
    bool in_cluster = false;
    bool back_needed = false;
    if (this->points[0].r < 10000){
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
    std::vector<PolarPoint>* buffer_cluster = new std::vector<PolarPoint> [this->nb_cluster+1];
    
    int i,j,size;

    for (i = 0; i<NB_POINT_SCAN; i++){
        if (this->points[i].cluster_id != -1){
            buffer_cluster[this->points[i].cluster_id].push_back(this->points[i]);
        }
    }
    
    int radius;
    float r, theta;

    // Solve the problem of a first cluster shared between the end and the beginning of the scan 
    if (this->nb_cluster>=0 && this->points[NB_POINT_SCAN-1].cluster_id == 0){
        i = buffer_cluster[0].size() - 1;
        while (buffer_cluster[0][i].cluster_id == 0) {
            buffer_cluster[0][i].angle -= NB_POINT_SCAN;
            i--;
        }
    }

    for (i = 0; i<this->nb_cluster; i++){
        r = 0;
        size = buffer_cluster[i].size();
        theta = 0;
        for (j = 0; j<size; j++){
            r = std::max(r,buffer_cluster[i].at(j).r);
            theta += buffer_cluster[i].at(j).angle;
        }
        theta *= 6.283185307179586/(NB_POINT_SCAN * size);
        r /= 1000;
        radius = 2 * sin(size/(2*NB_POINT_SCAN)) * r;
        
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

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClusterDetector>());
    rclcpp::shutdown();
    return 0;
}
