#include "point_cloud/pcl_manager.h"

PCLManager::PCLManager(): pcl_(new pcl::PointCloud<pcl::PointXYZ>){
    ros::Rate loop_rate(1);
    nh_ = ros::this_node::getName();
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pcd_topic", 1);
    filename_pub_ = nh_.advertise<std_msgs::String>("filename", 1);
    degree_sub_ = nh_.subscribe("/rotation_degree", 10, &PCLManager::rotationCallback, this);
    pcl_service_ = nh_.advertiseService("save_rotated_pcd", &PCLManager::saveRotatedPCD, this);

    deg_ = 0.0;
    prev_deg_ = 0.0;
    wait_count_ = 5;
    publish_ = false;
    transform_ = 1.0;

    // default value
    pcd_file_ = "wolf.pcd";
    scale_ = 2;
};

PCLManager::~PCLManager(){

};

void PCLManager::rotationCallback(const std_msgs::Float32::ConstPtr& msg){
    this->deg_ = msg->data;
    publish_ = true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCLManager::rotateYAxiz(float degree, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl){
    float rad = deg2Rad(degree);
    Eigen::Matrix4f rotation_matrix_y = Eigen::Matrix4f::Identity();
    rotation_matrix_y(0, 0) = cos(rad);
    rotation_matrix_y(0, 2) = sin(rad);
    rotation_matrix_y(2, 0) = -sin(rad);
    rotation_matrix_y(2, 2) = cos(rad);

    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_pcl(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(*pcl, *rotated_pcl, rotation_matrix_y);

    return rotated_pcl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCLManager::rotateYAxizV2(float degree, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl){
    float rad = deg2Rad(degree);
    float rotation_matrix_y[9] = {cos(rad), 0.0, sin(rad), 0.0, 1.0, 0.0, -sin(rad), 0.0, cos(rad)};

    int idx=0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_pcl (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < pcl->points.size(); ++i) {
        float x = pcl->points[i].x*rotation_matrix_y[idx] + pcl->points[i].y*rotation_matrix_y[idx+1] + pcl->points[i].z*rotation_matrix_y[idx+2];
        idx+=3 ;
        float y = pcl->points[i].x*rotation_matrix_y[idx] + pcl->points[i].y*rotation_matrix_y[idx+1] + pcl->points[i].z*rotation_matrix_y[idx+2];
        idx+=3; 
        float z = pcl->points[i].x*rotation_matrix_y[idx] + pcl->points[i].y*rotation_matrix_y[idx+1] + pcl->points[i].z*rotation_matrix_y[idx+2];
        idx=0;

        rotated_pcl->points.push_back(pcl::PointXYZ(x, y, z));
    }

    return rotated_pcl;
}

int PCLManager::initialize(){
    pcl::PCLPointCloud2 cloud_blob;

    if (pcl::io::loadPCDFile (filepath_ + pcd_file_, cloud_blob) == -1) // load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    pcl::fromPCLPointCloud2 (cloud_blob, *pcl_); // convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
    std::cout << "Loaded "
                << pcl_->width * pcl_->height
                << " data points from test_pcd.pcd with the following fields: "
                << std::endl;

    // find the maximum value among x, y, and z
    float max_value = std::numeric_limits<float>::lowest(); // initialize to the lowest possible float value
    for(auto& point: *pcl_){
        max_value = std::max({max_value, point.x, point.y, point.z});
    }

    // remember the transform value to convert it back to its initial size when saving
    transform_ = scale_ / max_value;

    // display
    for (auto& point: *pcl_){
        point.x = point.x * transform_;
        point.y = point.y * transform_;
        point.z = point.z * transform_;
        std::cout << "    " << point.x
                << " "    << point.y
                << " "    << point.z << std::endl;
    }

    return 0;
}

void PCLManager::processThread(){
    int count=0;
    ros::Rate loop_rate(1);

    initialize();

    while (ros::ok()) {
        if(count>=wait_count_){ //wait for the pcd get displayed on rviz
            if(publish_){
                //rotate
                rotated_pcl_ = rotateYAxiz(deg_, pcl_);
                prev_deg_ = deg_;

                //convert into msg
                pcl::toROSMsg(*rotated_pcl_, pcl_output_);
                
                pcl_output_.header.frame_id = "map";
                pcl_pub_.publish(pcl_output_);

                publish_ = false;
            }
        }
        else{
            count++;
            std::cout<<"WAITING"<<count<<'\n';
            //convert into msg
            pcl::toROSMsg(*pcl_, pcl_output_);

            pcl_output_.header.frame_id = "map";
            pcl_pub_.publish(pcl_output_);
        }
        std_msgs::String filename;
        filename.data = pcd_file_;

        filename_pub_.publish(filename);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool PCLManager::saveRotatedPCD(point_cloud::SavePCD::Request &req, point_cloud::SavePCD::Response &res){
    for (auto& point: *rotated_pcl_){
        point.x = point.x / transform_;
        point.y = point.y / transform_;
        point.z = point.z / transform_;
        std::cout << "    " << point.x
                << " "    << point.y
                << " "    << point.z << std::endl;
    }
    if (pcl::io::savePCDFileASCII(req.filename, *rotated_pcl_) == -1) {
        res.success = false;
        res.message = "Failed to save the PCD file.";
        return false;
    }

    res.success = true;
    res.message = "PCD file saved successfully.";
    return true;
}

void PCLManager::insertFile(std::string file_name, std::string filepath, int scale){
    this->pcd_file_ = file_name;
    this->filepath_ = filepath;
    this->scale_ = scale;
}

float PCLManager::deg2Rad(float degree){
    return degree/180 * M_PI;
}

std::string PCLManager::getFilename(){
    return this->pcd_file_;
}

std::string PCLManager::getFilePath(){
    return this->filepath_;
}

int PCLManager::getScale(){
    return this->scale_;
}

void PCLManager::setRotatedPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl){
    this->rotated_pcl_ = pcl;
}
