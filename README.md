# Overview
Autonomous driving car simulator based on gazebo.
The following sensors are simulated.
- camera
- imu
![screenshot from 2019-01-11 00-42-10-min](https://user-images.githubusercontent.com/8327598/50984661-bed3f780-1545-11e9-9af6-071ddd1cec76.png)

# Requirements
- ROS (higher kinetic)
- gazebo (higher version 7)

# Download
```shell
$ cd <catkin workspace/src/>
$ git clone https://github.com/yukkysaito/vehicle_sim.git --recurse-submodules
```
or if you already have a copy of the repo, run `$ git submodule update --init --recursive`.

# How to use

1. build

```shell
$ rosdep install --from-paths <vehicle_sim path> -y
$ cakin_make
$ source "your catkin workspace"/devel/setup.bash
```
3. launch gazebo

```shell
$ roslaunch vehicle_sim_launcher dock2rear.launch
```

If GPU is available

```shell
$ roslaunch vehicle_sim_launcher dock2rear.launch gpu:=true
```
[![](https://img.youtube.com/vi/JViNKB_igI4/0.jpg)](https://www.youtube.com/watch?v=JViNKB_igI4)

# Some example
## **Citysim** : http://gazebosim.org/blog/car_sim
```
$ roslaunch vehicle_sim_launcher gazebo7_citysim.launch gpu:=true
```


If you use gazebo9, simulate traffic lights and moving objects.
Build according to the [readme](https://github.com/yukkysaito/osrf_citysim/tree/9356b76bd827a3afcb71000b9274e3f64713a77c) and execute the following command
```
$ roslaunch vehicle_sim_launcher gazebo9_citysim.launch gpu:=true
```

![screenshot from 2019-01-11 00-40-35-min](https://user-images.githubusercontent.com/8327598/50985197-19ba1e80-1547-11e9-98d1-284b3172c064.png)
## **mcity(car_demo)** : https://github.com/osrf/car_demo
```
$ roslaunch vehicle_sim_launcher gazebo_mcity.launch gpu:=true
```
![screenshot from 2019-01-11 00-38-49-min](https://user-images.githubusercontent.com/8327598/50985258-3e15fb00-1547-11e9-91d4-3b826b82136e.png)

## **Connect to Autoware** : https://github.com/CPFL/Autoware
If you need pointcloud map and path files, you can [download](https://drive.google.com/open?id=1yu8s885HDkJp3IbMV06KWim2ZdUxIoIF).  
The follwing video is used *autoware_world/pointcloud_map* and *autoware_world/path*.
```
$ roslaunch vehicle_sim_launcher gazebo_autoware.launch gpu:=true
```
[![](https://img.youtube.com/vi/wIzZ25XJI2M/0.jpg)](https://www.youtube.com/watch?v=wIzZ25XJI2M)


# How to change vehicle info
You can customize sensor position and vehicle info.
- sensor position: vehicle/vehicle_model/config/caibration.yaml
- vehicle info: vehicle/vehicle_model/config/vehicle_info.yaml

# Aruco note
When I printed the front grid board coordinates, the markers were marked
in OpenCV coordinates like this, suggesting that the aruco board is rotated
half turn about X+
```	
        ROS				 OpenCV
                Z+             Z+
                |    X+       /
                |    /       /
                |   /       /
                |  /       /
                | /       /
                |/       /
Y+ -------------O       O--------------------------------------------------> X+
                :	| upside down 3 | upside down 4 | upside down 5 | 
                :	|---------------+---------------+---------------+
                :	| upside down 0 | upside down 1 | upside down 2 |
                :	|-----------------------------------------------+
                Z-	Y+
```
The points obtained from GridBoard are (in CV coordinate)
(0.00, 0.31, 0.00) (0.15, 0.31, 0.00) (0.15, 0.16, 0.00) (0.00, 0.16, 0.00)
(0.16, 0.31, 0.00) (0.31, 0.31, 0.00) (0.31, 0.16, 0.00) (0.16, 0.16, 0.00)
(0.32, 0.31, 0.00) (0.47, 0.31, 0.00) (0.47, 0.16, 0.00) (0.32, 0.16, 0.00)
(0.00, 0.15, 0.00) (0.15, 0.15, 0.00) (0.15, -0.00, 0.00) (0.00, -0.00, 0.00)
(0.16, 0.15, 0.00) (0.31, 0.15, 0.00) (0.31, -0.00, 0.00) (0.16, -0.00, 0.00)
(0.32, 0.15, 0.00) (0.47, 0.15, 0.00) (0.47, -0.00, 0.00) (0.32, -0.00, 0.00)

markers (5,2) in cam2; T = [-1.59, 0.03, 1.11] R = [-2.80, -0.04, -1.24] 
cam2 (1.11, 1.59); Q(0.04, 0.01, -0.91, 0.40) = [0.04, 0.01, -1.00] 2.31
markers (3,0) in cam3; T = [1.17, 0.03, 1.55] R = [-2.86, 0.02, 1.17] 
cam3 (1.55, -1.17); Q(0.02, -0.01, -0.93, -0.38) = [0.03, -0.01, -1.00] 3.92

## Maruco
markers (5,2) in cam2; T = [-1.52, -0.14, 1.68] R = [-0.04, 2.93, 0.06] 
cam2 (1.68, 1.52); Q(0.02, 0.02, -0.99, 0.10) = [0.02, 0.02, -1.00] 2.93
markers (3,0) in cam3; T = [1.40, -0.15, 1.79] R = [-0.06, -3.00, -0.09] 
cam3 (1.79, -1.40); Q(-0.03, 0.02, 1.00, 0.07) = [-0.03, 0.02, 1.00] 3.00
-------------------------------------------------------
front = [0.375, -0.255, 0.15] [0.375, -0.085, 0.15] [0.375, -0.085, 0] [0.375, -0.255, 0]
[0.375, -0.085, 0.15], [0.375, 0.085, 0.15], [0.375, 0.085, 0], [0.375, -0.085, 0]
[0.375, 0.085, 0.15], [0.375, 0.255, 0.15], [0.375, 0.255, 0], [0.375, 0.085, 0]
[0.375, -0.255, 0], [0.375, -0.085, 0], [0.375, -0.085, -0.15], [0.375, -0.255, -0.15]
[0.375, -0.085, 0], [0.375, 0.085, 0], [0.375, 0.085, -0.15], [0.375, -0.085, -0.15]
[0.375, 0.085, 0], [0.375, 0.255, 0], [0.375, 0.255, -0.15], [0.375, 0.085, -0.15]

right
[0.375, 0.255, 0.15], [0.1875, 0.255, 0.15], [0.1875, 0.255, 0], [0.375, 0.255, 0]
[0.1875, 0.255, 0.15], [0, 0.255, 0.15], [0, 0.255, 0], [0.1875, 0.255, 0]
[0, 0.255, 0.15], [-0.1875, 0.255, 0.15], [-0.1875, 0.255, 0],	[0, 0.255, 0]
[-0.1875, 0.255, 0.15], [-0.375, 0.255, 0.15], [-0.375, 0.255, 0], [-0.1875, 0.255, 0]
[0.375, 0.255, 0], [0.1875, 0.255, 0], [0.1875, 0.255, -0.15], [0.375, 0.255, -0.15]
[0.1875, 0.255, 0], [0, 0.255, 0], [0, 0.255, -0.15], [0.1875, 0.255, -0.15]
[0,	0.255, 0], [-0.1875, 0.255, 0], [-0.1875, 0.255, -0.15], [0, 0.255, -0.15]
[-0.1875, 0.255, 0], [-0.375, 0.255, 0], [-0.375, 0.255, -0.15], [-0.1875, 0.255, -0.15]

rear
[-0.375, 0.255, 0.15], [-0.375,	0.085, 0.15], [-0.375, 0.085, 0], [-0.375, 0.255, 0]
[-0.375, 0.085,	0.15], [-0.375, -0.085, 0.15], [-0.375, -0.085, 0], [-0.375, 0.085, 0]
[-0.375, -0.085, 0.15], [-0.375, -0.255, 0.15],	[-0.375, -0.255, 0], [-0.375, -0.085, 0]
[-0.375, 0.255,	0], [-0.375, 0.085,	0], [-0.375, 0.085, -0.15], [-0.375, 0.255, -0.15]
[-0.375, 0.085,	0],	[-0.375, -0.085, 0], [-0.375, -0.085, -0.15], [-0.375, 0.085, -0.15]
[-0.375, -0.085, 0], [-0.375, -0.255, 0], [-0.375, -0.255, -0.15], [-0.375, -0.085, -0.15]

left
[-0.375, -0.255, 0.15], [-0.1875, -0.255, 0.15], [-0.1875, -0.255, 0], [-0.375, -0.255, 0]
[-0.1875, -0.255, 0.15], [0, -0.255, 0.15], [0, -0.255, 0], [-0.1875, -0.255, 0]
[0, -0.255, 0.15], [0.1875, -0.255, 0.15], [0.1875, -0.255, 0], [0, -0.255, 0]
[0.1875, -0.255, 0.15], [0.375, -0.255, 0.15], [0.375, -0.255, 0], [0.1875, -0.255, 0]
[-0.375, -0.255, 0], [-0.1875, -0.255, 0], [-0.1875, -0.255, -0.15], [-0.375, -0.255, -0.15]
[-0.1875, -0.255, 0], [0, -0.255, 0], [0, -0.255, -0.15], [-0.1875, -0.255,	-0.15]
[0, -0.255,	0], [0.1875, -0.255, 0], [0.1875, -0.255, -0.15], [0, -0.255, -0.15]
[0.1875, -0.255, 0], [0.375, -0.255, 0], [0.375, -0.255, -0.15], [0.1875, -0.255, -0.15]
