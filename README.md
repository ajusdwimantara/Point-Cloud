
# Knowledge Test


## 1. Counting Prime Numbers In A Fibonacci Series

### Usage
- Go to the file (1.cpp) directory 
- Compile the program ```g++ 1.cpp -o 1```
- Run the program ```./1```

### Demo
![App Screenshot](/screenshots/screenshot-1.png)

## 2-4. Point Cloud Handling

### Usage
- Move the `point_cloud` package into `<your_workspace/src>` directory
- In `your_workspace` directory, build using `catkin_make`
- In `your_workspace` directory, source using `source devel/setup.bash`
- Launch the main node using `roslaunch point_cloud pcl_node.launch`, A user input GUI will appear an can be used to input angle of rotation (in degree) to rotate the point cloud in the Y axis.

##### note: the launch file is able to take several arguments 
`roslaunch point_cloud pcl_node.launch filename:=desired_pcd_file filepath:=/path/to/your/pcd_data/ scale:=integer_based_on_desired_scale unit_test:=true_or_false`

##### but if you doesn't specify any arguments, the default value (in the pcl_node.launch file) will be used.

- Run the `save_pcd_client` on another terminal `rosrun point_cloud save_pcd_client /path/to/your/pcd_data/`

### Demo
![App Screenshot](/screenshots/screenshot-2.png)
