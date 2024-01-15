# joycon_teleop

A ROS 2 python package for using the IMU data from Nintendo Joycons to control the a robot with end-effector velocity control

# Dev notes

So far, I've got the device working, and I'm looking at accel and gyro data.
The gyro data looks pretty good, so estimating rotation velocity works pretty well.
But the accel data is pretty noisy, and I do not think is good enough to use for estimating linear velocity.
