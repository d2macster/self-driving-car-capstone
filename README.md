# Programming a Real Self-Driving Car:
## System Integration Project

[image1]: ./imgs/Carla.png "Image 1"   
[image2]: ./imgs/project-rosgraph.png "Image 2"   

This is the project repo of team Laters for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. 

The goal of the project is to fully implement with ROS the main modules of an autonomous vehicle: Perception, Planning and Control, which will be tested on Udacity´s Self Driving Car _´Carla´_ around a test track using waypoint navigation. 

For validation the code was tested using a simulator where the car drives around a highway test track with traffic lights.

## Carla: The Test Car
![alt text][image1]

As the first step, the project will be evaluated in a speedway simulator.
Here is the recording of car performance.
 [![simulator](https://img.youtube.com/vi/NnkCm7HQY_A/0.jpg)](https://youtu.be/NnkCm7HQY_A)

The project was also evaluated on Carla, an autonomous Lincoln MKZ, at Udacity´s test site in Palo Alto, California. 
Here is the recording of car performance on the test parking lot.
 [![simulator](https://img.youtube.com/vi/v5yzrBtW-q4/0.jpg)](https://youtu.be/v5yzrBtW-q4)

The operating system Carla runs on is Ubuntu Linux. 

Udacity Self-Driving Car Harware Specs:
* 31.4 GiB Memory
* Intel Core i7-6700K CPU @ 4 GHz x 8
* TITAN X Graphics
* 64-bit OS

## The Team: Laters

|           | Name                     |    E-Mail                        |      GitHub                                     |
| --------- | -------------------------| -------------------------------- | :----------------------------------------------:|
| Team Lead | Andrii Cherniak          |    theandriicherniak@gmail.com   |      [Andrii](https://github.com/d2macster)     |
|           | Tharatch Sirinukulwattana|    tharatch.siri@gmail.com       |      [Tharatch](https://github.com/TharatchSiri)|
|           | Melanie Plaza            |    melanieplaza@gmail.com        |      [Melanie](https://github.com/mplaza)       |
|           | Alexander Epifanov       |    wave911@yandex.ru             |      [Alexander](https://github.com/wave911)    |
|           | Igor Molina              |    elchacho@gmail.com            |      [Igor](https://github.com/igolas0)         |

## Project Overview

In the figure below you can find an overview of the main software components and how they communicate with each other and with the car or simulator.

Carla is equipped with a drive-by-wire system (DBW) and hence the throttle, brake and steering can be electronically controlled. In the graph below the Control Module outputs Throttle, Brake and Steering signal commands to the car or simulator. These commands are set to be published at 50Hz since this is the frequency that Carla´s DBW system expects.

![alt text][image2]



## Approach and Code Description

The submitted code is implemented in ROS. For this project we mainly use __rospy__, which is a pure Python client library for ROS and enables Python programmers to interface with ROS Topics, Services and Parameters.

We proceed to describe the modules and main components using the following structure:

1. Perception Module
     * 1.1 Traffic Light Detection Node
     
2. Planning Module
      * 2.1 Waypoint Loader
      * 2.2 Waypoint Updater
      
3. Control Module
      * 3.1 DBW Node
      * 3.2 Waypoint Follower

#### 1.1 Traffic Light Detection Node (Perception Module)

The Perception Module consists of a Traffic Light Detection Node. For now we are skipping the Obstacle Detection Node since currently we do not dispose of Lidar and Radar data when testing on Carla. Hence it would be very difficult with current available hardware and measurements to build a realiable obstacle detector.

As a classifier for the traffic detection node we use a CNN model for light detection on the simulator. For light detection on real images and testing on Carla we trained a larger model which is more computationally expensive and requires powerful hardware like the one installed on Carla. 

The Traffic Light Detection Node is implemented [here](./ros/src/tl_detector/tl_detector.py).

After initialization this Node subscribes to the following topics: `/current_pose`, `/base_waypoints`, `/image_color` and `/car_waypoint_id`  (code lines 30 to 33). Then, after defining some important variables, we search for the waypoint of the first traffic lights stop ahead (code lines 62 to 64). 

Next the function `image_cb()` (lines 78 to108) identifies red lights in the incoming camera image and publishes the index of the waypoint closest to the red light's stop line to /traffic_waypoint. It publishes upcoming red lights at camera frequency. Each predicted state has to occur `STATE_COUNT_THRESHOLD` number of times (defined in line 16) until we start using it. Otherwise the previous stable state is used.

To achieve this the helper function `process_traffic_lights()` is used (starting with line 144), which finds closest visible traffic light, if one exists, and determines its location and color. It returns the integer index of the waypoint closest to the upcoming stop line at a traffic light (-1 if none exists) and the color index of the detected traffic light if any (0: RED, 1: YELLOW, 2: GREEN, 4: UNKNOWN). 

In line 142 we also make use of the imported `TL_classifier` function inside of the `tl_classifier.py` script which you can find  [here](./ros/src/tl_detector/light_classification/tl_classifier.py). It loads with Keras the pre-trained model to classify the incoming images and returns the color index of the detected color or returns unknown if nothing is detected.

This will make up for the Perception Module. Next we will visit the Planning Module.
 
#### 2.1 Waypoint Loader (Planning Module)

The Waypointer Loader Node is implemented in [./ros/src/waypoint_loader/waypoint_loader.py](./ros/src/waypoint_loader/waypoint_loader.py). There the node gets initizialized and publishes to the `/base_waypoints` topic messages in the `Lane` format defined in [./ros/src/styx_msgs/msg/Lane.msg](./ros/src/styx_msgs/msg/Lane.msg).
  
In a nutshell the `waypoint_loader` Node loads the programmed waypoints for the car to follow around the track (line 26) and the default cruising speed (via the ROS parameter `~velocity` - see line 25). In line 26 the function `new_waypoint_loader()` is called which loads and publishes the waypoints with the default cruise velocity attached to them. 
  
Also inside of `waypoint_loader` additional helper functions (mainly for unit conversions) are defined and used. The published `/base_waypoints` topic will be used by the `waypoint_updater` Node, which will search and filter the relevant waypoints to follow ahead of the vehicle and update target velocities to follow depending on the current driving strategy. `base_waypoints` is also used by the perception module.

#### 2.2 Waypoint Updater (Planning Module)

This [node](./ros/src/waypoint_updater/waypoint_updater.py) will publish waypoints from the car's current position to some `x` distance ahead. It will also consider traffic lights and set target speeds (attached to waypoints) accordingly to be able to adapt: meaning accelerate, stop at traffic lights or cruise depending of the data coming from the Perception Module.

We initialize the node (line 34) and subscribe to `/current_pose`, `/base_waypoints`, `/traffic_waypoints` and `/current_velocity` topics (lines 37 to 40). Then in lines 48 to 50 we define the topics we want to publish: `/final_waypoints` for the Control Module, `/driving_mode_pub` and `/car_waypoint_id_pub`. Also in lines 27 to 29 we already defined and set important parameters. `LOOKAHEAD_WPS` determines the number of waypoints ahead of the vehicle to be published for the Control Module. `BUFFER` will be used to parametrize the deceleration process just before stopping at red lights. The car will decelerate smoothly and cruise at 3mph the final yards until the buffer distance before the stopping line is reached and then finally stops.

From line 54 to 69 we initialize further relevant parameters. In line 74 & 75 we call `current_velocity_cb()` and query the current velocity when accessing the subscribed topic `/current_velocity.` Then in lines 77 to 81 function `pose_cb()` returns the current position of the car in the environment when querying the `/current_pose` topic and publishes the previously described topics to be published in this node. Furthermore `waypoints_cb()` loads the basic path to follow coming from the `waypoint_loader` Node only if it has not been done already (loads only once) and `traffic_cb()` returns the index of the waypoint to stop at if a red or yellow light is detected (this information comes from Perception via the `/traffic_waypoint` topic).

Next from lines 100 to 193 we define some helper functions which we will not describe in detail, but we proceed to explain the main logic in this node which happens in `prepare_lookahead_waypoints()`. 

First we access the accelaration and deceleration limits via ROS parameters  (lines 198 to 201). Then in lines 203 to 216 we calculate the distance from current position to the next waypoint ahead and adjust the target velocity accordingly. For this purpose we use a PID Controller (implemented [here](./ros/src/waypoint_updater/pid.py)) to slow down the car if it deviates much from the trajectory defined by the waypoints, like passing a sharp turn. After that we set the array `next_waypoints` to the 200 waypoints (or whatever the number we set the variable `LOOKAHEAD_WPS` to) of the path that are directly situated ahead of the car (lines 219 to 221).

From lines 223 to 241 we define how to behave based on upcoming traffic light detections. If no red or yellow lights are detected (in a reasonable distance in front of the car) we will proceed to speed up without violating the incoming cruising speed via ROS parameter or max speed limit. Else we will decelerate and stop at the stopping line if the braking distance is long enough to guarantee a safe manouver inside the deceleration limit range.

The speed up behaviour is defined by the function `speed_up()` (lines 243 to 254) and the deceleration by function `slow_down()` (lines 257 to 277).

Finally the function `publish_final_waypoints()` which is called from `pose_cb` helps to publish the final waypoints, the `driving_mode_pub` (braking or not braking) and the waypoint representing the car´s current position. The last two do not appear in the overview figure in section "Project Overview".

Next and last stop of our overview will be the nodes inside of the Control Module, which sends the proper commands to the vehicle actuators.

#### 3.1 DBW Node (Control Module)

The DBW Node is implemented in [./ros/src/twist_controller/dbw_node.py](./ros/src/twist_controller/dbw_node.py) and here is where the steering, throttle and braking signal commands get published to the car or simulator.

For this a twist controller is imported which is implemented [here](./ros/src/twist_controller/twist_controller.py). The twist controller (including the imported [yaw controller](./ros/src/twist_controller/yaw_controller.py)) manages to set the desired linear and angular velocity with the help of a [PID controller](./ros/src/twist_controller/pid.py) and a [Low Pass Filter](./ros/src/twist_controller/lowpass.py) which outputs the necessary actuator signals. We subscribe to the desired linear and angular velocity via the `twist_cmd` topic which is published by the `Waypoint Follower` Node.

In order to mimic human driving and prevent ping-pong actioning of throttle and brake combinations we pass a variable specifying driving mode of the car ( acceleration / deceleration) and relax the precision of following suggested speed. E.g. if we are accelerating and the car  speed increased a bit more than expected the controller will just release throttle and let the car cruise so it can slow down on its own (lines 91-108). We then also mimic brake booster functionality in lines 120-128. When the desired speed is very low we will activate extra braking force. Moreover if the car almost stopped we will apply full braking to keep the car from moving.

Finally, since a safety driver may take control of the car during testing we consider the DBW status in our implementation which can be found by subscribing to `/vehicle/dbw_enabled`.

#### 3.2 Waypoint Follower (Control Module)

For the `Waypoint Follower` Node we make use of a package containing code from [Autoware](https://github.com/CPFL/Autoware) which subscribes to `/final_waypoints` and publishes target vehicle linear and angular velocities in the form of twist commands to the `/twist_cmd` topic. You can find the code [here](./ros/src/waypoint_follower/src/pure_pursuit.cpp). 

## Project Results

Following the approach detailed above we get pretty satisfactory results when testing in the simulator and will proceed to submit our work for the real test on Carla!

The car successfully drives around the simulator highway and test lot tracks with proper stopping at traffic lights. Rarely some accidents happen, but we attribute those incidents solely to ocassional CPU overloads which cause the communication between the ROS controller and the simulator to work suboptimally. Here lies potentially further room for efficiency optimization of the code, which we have done partially already (see next section) or making sure that enough hardware capacity is guaranteed.

Finally we want to thank Udacity for this great opportunity and learning experience and all team members. We are very happy to graduate and happily await even more awesome future career paths.

### Optimization of Code Running Time Efficiency

For automotive applications it is specially important to develop very efficient software solutions since hardware capacity on board is scarce. Powerful hardware drives the car price up and increases fuel consumption (or decreases the electric vehicle´s range) because of higher weight or higher electrical demands on the supply-system.

Furthermore inefficient code presents the risk of overloading the hardware system and potentially "breaking", hence represents a very serious safety issue for autonomous driving. In order to address this one needs to implement redundant hardware and software system solutions, but the first measure should always be to write clean, efficient and robust code.

In the scope of this project we have taken several measures to improve the efficiency of our code. Here are some examples:

1. __Algorithm optimization:__ In `waypoint_updater.py`our first implementation kept re-using the previous waypoint match when computing the closest waypoint to the car. We optimized our algorithm so that it does not make unnecessary calls.
2. __Avoid re-computing__: Our Traffic Light Detection Node was computing similar variables that were already being computed in `waypoint_updater`. Hence we made `waypoint_updater` publish the car waypoint position to the `/car_waypoint_id` topic and the traffic light detector Node read from that topic. 
3. __Avoiding unnecessary readings__: Subscribing to ROS topics can be computationally expensive since we have incoming data streams incoming on every loop. In the Traffic Light Detection Node we found a more efficient way to avoid subscribing to the `traffic_lights` topic by pre-computing traffic light classification.

---------------------------------------------------------------------------------------------
### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
