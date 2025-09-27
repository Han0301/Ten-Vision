## 1 Ubuntu and [ROS](https://www.ros.org/)**
```
sudo apt-get install ros-xxx-pcl-conversions
```

## **2 Eigen**
```
sudo apt-get install libeigen3-dev
```

## **3 livox_ros_driver**

https://github.com/Livox-SDK/livox_ros_driver

## 4. Build
```
    cd Point-LIO
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
```
## 5. run

```
    cd ~/$Point_LIO_ROS_DIR$
    source devel/setup.bash
    roslaunch point_lio mapping_avia.launch
    roslaunch livox_ros_driver livox_lidar_msg.launch
```
