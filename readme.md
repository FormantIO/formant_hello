
Running:

`roslaunch formant_hello navigation.launch map_yaml:=/home/hello-robot/stretch_user/maps/brians_basement.yaml`



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