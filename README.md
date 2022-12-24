# ros_sfm
Structure from motion for ros. Uses pcl and standard ros packages

## ⚙️ Setup
Dependeces (ROS, PCL, pcl_conversions, pcl_msgs)

1. Clone the GitHub repository into 'catkin_ws/src'
```shell
git clone https://github.com/jagennath-hari/ros_sfm.git
```
2. 'catkin build' and source the workspace.

## 🖼 Demo
Download [bag file](https://drive.google.com/uc?export=download&id=1SUDQQADDZAbozKulQ5Lv8tRqpfOsAcj8) and paste inside 'bag' folder in sfm.
To run a demo.
```shell
roslaunch sfm structure_from_motion_example.launch
```
## 🏁 To use


## 📊 Visualization
Rviz can be used to view the topics 'sfm/cloud' and 'sfm/trajectory', they are 'sensor_msgs/PointCloud2' and 'nav_msgs/Path' topic types respectively.
