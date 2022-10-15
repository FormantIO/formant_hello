
Running:

`roslaunch formant_hello navigation.launch map_yaml:=/home/hello-robot/stretch_user/maps/brians_basement.yaml`


Noetic Setup:

1. Install ROS noetic
1. add source of noetic script to .bashrc and source the file
1. mkdir ros_ws
1. cd ros_ws
1. mkdir src
1. catkin_make
1. cd src
1. clone stretch_ros
1. clone formant_hello [setup for github key first]
1. sudo apt install ros-noetic-move-base
1. rosdep update
1. rosdep install --from-paths src --ignore-src -r -y
1. cd ..
1. catkin_make
1. cd src/stretch_ros/stretch_description/urdf/
1. ./xacro_to_urdf.sh
1. cd ../../../..
1. sudo apt install ros-noetic-rplidar-ros
1. pip3 install pyyaml
1. pip3 install hello-robot-stretch-body
1. copied stretch_user folder over to new user
1. added export HELLO_FLEET_PATH=/home/NEW_USER/stretch_user and export HELLO_FLEET_ID=stretch-re1-1051 to .bashrc
1. source ~/.bashrc
1. cd src/formant_hello/
1. git checkout noetic
1. cd ../..
1. pip3 install hello-robot-stretch-body-tools
1. /home/cora/.local/bin/RE1_migrate_params.py
1. /home/cora/.local/bin/RE1_migrate_contacts.py
1. cd src/stretch_ros
1. git checkout dev/noetic
1. cd ../..
1. cp /home/cora/ros_ws/src/stretch_ros/stretch_core/config/controller_calibration_head_factory_default.yaml /home/cora/ros_ws/src/stretch_ros/stretch_core/config/controller_calibration_head.yaml
1. roslaunch formant_hello mapping.launch
1. (new terminal)
1. mkdir -p ~/stretch_user/maps
1. rosrun map_server map_saver -f ${HELLO_FLEET_PATH}/maps/<map_name>
1. (original terminal)
1. ctrl-c
1. sudo apt install ros-noetic-laser-scan-matcher
1. roslaunch formant_hello navigation.launch map_yaml:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml


Notes about Hello-Robot Stretch Integration:

1. Cannot seem to command translational and rotational velocity to the wheels at the same time, it seemingly only works on one
2. Stretch API frequntly starts up with RPC errors, just need to keep restarting until it homes correctly
3. Joint status does not seem to update from the API automatically, you must call `pull_status` on that motor to update that before reading the status
4. URDF for the `joint_lift` had incorrect axis for the prismatic action (was set to Z but due to rotation should be set to Y) resulting in translation horizontally instead of vertically
5. Only wheels support velocity commands, head, lift, extension, and wrist only allow for position
6. Default ROS node had increasing lag on scan topic, currently investigating mitigation techniques
7. No obvious way to "stop" a joint motion and is currently underway
8. SDK only supports python2, which was depreciated on Jan 1 2020
9. Navigation stack suffers greatly from wheel slip, does not appear to leverage SLAM techniques to minimize accumulated error
10. Appears as though the default motor controllers for the base are poorly tuned, results in poor path following from navigation commands
