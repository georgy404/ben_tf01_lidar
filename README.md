# ben_tf01_lidar

ROS package for [Benewake TF01 lidar](http://www.benewake.com/TF01_e.html) 

## 1. Get started

Run launch file in terminal:
 ``` 
$ roslaunch ben_tf01_lidar tf01_boardcast.launch
```

## 2. Nodes
#### 2.1 ben_tf01_lidar
Benewake TF01 lidar driver.
#### 2.1.1 Published Topics
/ir_height ([sensor_msgs/Range](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html) )
    Output distance information. 
#### 2.1.2 Parameters
port (str) 
    port to which lidar is connected (example: /dev/ttyUSB0)
frame (str) 
    [TF](http://wiki.ros.org/tf)  frame for lidar relative to which will measured length

