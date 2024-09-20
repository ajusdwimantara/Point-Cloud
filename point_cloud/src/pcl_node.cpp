#include "point_cloud/pcl_manager.h"
#include "point_cloud/unit_test.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_node");
    ros::NodeHandle nh(ros::this_node::getName());

    std::string pcd_file, filepath;
    int scale;
    bool unit_test;

    // get the param
    nh.getParam("filename", pcd_file);
    nh.getParam("scale", scale);
    nh.getParam("filepath", filepath);
    nh.getParam("unit_test", unit_test);

    if(unit_test){
        testRotateYAxiz();
        testRotateYAxizV2();
        testSaveRotatedPCD(filepath);
        testInsertFile(filepath);
        testDeg2Rad();
        while(ros::ok()){
            ros::spinOnce();
        }
    }
    else{
        PCLManager* manager = new PCLManager();
        manager->insertFile(pcd_file+".pcd", filepath, scale);
        manager->processThread();
        delete manager;
    }

    
    return 0;
}