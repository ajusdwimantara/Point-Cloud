#include <ros/ros.h>
#include <std_msgs/String.h>
#include "point_cloud/SavePCD.h"
#include <stdio.h>
#include <unistd.h>
#include <string>


bool ready = false;
std::string filename;

void filenameCallback(const std_msgs::String::ConstPtr& msg) {
  filename = msg->data;
  ready = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "save_rotated_pcd_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<point_cloud::SavePCD>("/point_cloud_node/save_rotated_pcd");
    ros::Subscriber sub = nh.subscribe("point_cloud_node/filename", 10, filenameCallback);
    point_cloud::SavePCD srv;
    std::string dir = "";

    // wait for the filename
    while(!ready && ros::ok()){
      ros::spinOnce();
    }

    if (argc > 1) { // parse the argument
        dir = argv[1];
    }

    std::string path = dir + "/rotated_" + filename;
    srv.request.filename = path;

    char inp = 'y';
    while(inp=='y'||inp=='Y'){
      std::cout<<"Save rotated point cloud? (y/n): ";
      std::cin>>inp;
      if(inp!='y' && inp!='Y'){
        break;
      }
      if (client.call(srv)) {
          if (srv.response.success) {
              ROS_INFO("Success: %s", srv.response.message.c_str());
          } else {
              ROS_ERROR("Failed: %s", srv.response.message.c_str());
          }
      }
      else {
          ROS_ERROR("Failed to call service save_rotated_pcd");
      }
      ros::spinOnce();
    }

    return 0;
}
