<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="#">
    <img src="https://avatars.githubusercontent.com/u/72953705?s=200&v=4" alt="xLogo" width="80" height="80"></br></br>
    <img src="http://wiki.ros.org/custom/images/ros_org.png" alt="DockerLogo" width="200" height="50">
  </a>

<h2 align="center">ROS wrapper</h2>

The ROS wrapper for collaborative multi sensors odometry using a hybrid SLAM-MSCKF tightly-coupled approach.

See the [xWiki](https://github.com/jpl-x/x/wiki) for more information
about [x_vio_ros](https://github.com/jpl-x/x/wiki/xvio-ros).

This is the code for the paper **Data-Efficient Collaborative Decentralized Thermal-Inertial Odometry**
([PDF](https://rpg.ifi.uzh.ch/docs/RAL22_Polizzi.pdf)) by [Vincenzo Polizzi](https://github.com/viciopoli01/)
, [Robert Hewitt](https://github.com/neorobo), [Javier Hidalgo-Carrió](https://github.com/jhidalgocarrio)
, [Jeff Delaune](https://github.com/jeff-delaune) and [Davide Scaramuzza](http://rpg.ifi.uzh.ch/people_scaramuzza.html).
For an overview of our method, check out our [webpage](https://rpg.ifi.uzh.ch/xctio.html).

If you use any of this code, please cite the following publication:

```bibtex
@ARTICLE{Polizzi22RAL,
  author={Polizzi, Vincenzo and Hewitt, Robert and Hidalgo-Carrió, Javier and Delaune, Jeff and Scaramuzza, Davide},
  journal={IEEE Robotics and Automation Letters},   
  title={Data-Efficient Collaborative Decentralized Thermal-Inertial Odometry},   
  year={2022},  
  volume={7},  
  number={4},  
  pages={10681-10688},  
  doi={10.1109/LRA.2022.3194675}
}
```

## Usage

### Requirements

- Install the [x library](https://github.com/jpl-x/x_multi_agent)
- ROS [Noetic](http://wiki.ros.org/noetic)

### Build

Clone, build and source the ROS package

```bash
$ mkdir -p x_vio_ws/src && cd x_vio_ws/src
$ git clone git@github.com:jpl-x/x_multi_agent_ros.git
$ cd ..
$ catkin build x_vio_ros -DCMAKE_BUILD_TYPE=Release
$ source devel/setup.bash
```

### Launch

Create the ROS workspace as in the wiki then run:

- Single VIO visual data

```sh
$ roslaunch x_vio_ros vio.launch
```

- Multi VIO visual data

```sh
$ roslaunch x_vio_ros vio_multi.launch nr:=N # N is the number of UAVs you want to launch
```

- Single VIO thermal data

```sh
$ roslaunch x_vio_ros vio_multi_thermal.launch nr:=1
```

- Multiple VIO thermal data

```sh
$ roslaunch x_vio_ros vio_multi_thermal.launch nr:=2
```

### Visualization

The visualization is made out of the [RViz](http://wiki.ros.org/rviz) and [rqt](http://wiki.ros.org/rqt) tools from the
ROS software stack.

Supported visualization for two UAVs:

```bash
$ roscd x_vio_ros/scripts/ && ./load_gui_duo.sh
```

### Datasets

Download the bag files from [here](https://rpg.ifi.uzh.ch/xctio.html).

- Run the thermal dataset.

```bash
$ rosbag play mars_yard_duo_tf.bag --clock -s 9 -u 41 -r 0.1
```

- Inveraray castle parallel with four UAVs

```sh
$ rosbag play inveraray_around_4.bag --clock /UAV0/cam0/image_raw:=/UAV0/image_raw /UAV0/imu0:=/UAV0/imu /UAV0/ground_truth_pose:=/UAV0/true_pose /UAV1/cam0/image_raw:=/UAV1/image_raw /UAV1/imu0:=/UAV1/imu /UAV1/ground_truth_pose:=/UAV1/true_pose /UAV2/cam0/image_raw:=/UAV2/image_raw /UAV2/imu0:=/UAV2/imu /UAV2/ground_truth_pose:=/UAV2/true_pose /UAV3/cam0/image_raw:=/UAV3/image_raw /UAV3/imu0:=/UAV3/imu /UAV3/ground_truth_pose:=/UAV3/true_pose -r 0.05
```

- Inveraray castle around with four UAVs

```sh
$ rosbag play inveraray_parallel.bag --clock /UAV0/cam0/image_raw:=/UAV0/image_raw /UAV0/imu0:=/UAV0/imu /UAV0/ground_truth_pose:=/UAV0/true_pose /UAV1/cam0/image_raw:=/UAV1/image_raw /UAV1/imu0:=/UAV1/imu /UAV1/ground_truth_pose:=/UAV1/true_pose /UAV2/cam0/image_raw:=/UAV2/image_raw /UAV2/imu0:=/UAV2/imu /UAV2/ground_truth_pose:=/UAV2/true_pose /UAV3/cam0/image_raw:=/UAV3/image_raw /UAV3/imu0:=/UAV3/imu /UAV3/ground_truth_pose:=/UAV3/true_pose -r 0.05
```

## Acknowledgments

    The research was funded by the Combat Capabilities Development Command Soldier
    Center and Army Research Laboratory. This research was carried out at the Jet 
    Propulsion Laboratory, California Institute of Technology, and was sponsored 
    by the JPL Visiting Student Research Program (JVSRP) and the National 
    Aeronautics and Space Administration (80NM0018D0004).
