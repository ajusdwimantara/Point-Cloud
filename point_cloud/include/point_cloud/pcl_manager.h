#ifndef PCL_MANAGER_H_
#define PCL_MANAGER_H_

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include "std_msgs/String.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <thread>
#include <condition_variable>
#include "point_cloud/SavePCD.h"

/*
    Rotation Matrix Y = [[cos , 0, sin ],
                         [0   , 1, 0   ],
                         [-sin, 0, cos]]
*/

class PCLManager {
private:
    // PCD File
    std::string pcd_file_;
    std::string filepath_;
    int scale_;
    float transform_;

    // Point Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_pcl_;
    float deg_, prev_deg_;

    int wait_count_;
    char choose_;

    // ROS
    ros::NodeHandle nh_;
    ros::Publisher pcl_pub_;
    ros::Publisher filename_pub_;
    ros::Subscriber degree_sub_;
    ros::ServiceServer pcl_service_;

    sensor_msgs::PointCloud2 pcl_output_;

    bool publish_;
    
public:
    PCLManager();
    ~PCLManager();

    int initialize();
    void processThread();

    // Rotate Functions
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotateYAxiz(float degree, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotateYAxizV2(float degree, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl); //Without using pcl::transformPointCloud() function

    float deg2Rad(float degree);

    // Service
    bool saveRotatedPCD(point_cloud::SavePCD::Request &req, point_cloud::SavePCD::Response &res);

    // Subscriber
    void rotationCallback(const std_msgs::Float32::ConstPtr& msg);

    // Helper
    void insertFile(std::string file_name, std::string filepath, int scale);

    // Functions for unit test
    std::string getFilename();
    std::string getFilePath();
    int getScale();
    void setRotatedPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl);
};

#endif // PCL_MANAGER_H_