# ros_sfm
Structure from motion for ros. Uses pcl and standard ros packages

## ⚙️ Setup

1. Clone the GitHub repository into 'catkin_ws/src'
```shell
git clone https://github.com/jagennath-hari/ros_sfm.git
```
2. 'catkin build' and source the workspace.

## 🖼 Demo

To run a demo.
```shell
roslaunch sfm structure_from_motion_example.launch
```

## 📊 Visualization
Rviz can be used to view the topics 'sfm/cloud' and 'sfm/trajectory', they are 'sensor_msgs/PointCloud2' and 'nav_msgs/Path' topic types respectively.
