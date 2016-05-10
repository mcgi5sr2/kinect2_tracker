### kinect2_tracker
A working ROS wrapper for the KinectOne (v2) using libfreenect2

#install 
first install libfreenect2 from https://github.com/OpenKinect/libfreenect2/

this repository needs to be cloned into your /catkin_ws/src/ dir

git clone http://amrcgithub/mep12sr/kinect2_tracker

then simply catkin_make 

a symbolic link needs to be created between the .ros dir where the the launch file is executed from and the NiTE2 dir where the learning algorithm stores its data.
The NiTE2.bash file has an example of how to achieve this.

The libOpenni2.so library needs to be copied to your /usr/lib dir

To run the program the launch file needs to be used

roslaunch kinect2_tracker tracker.launch

##To Do
1. Set up quarternions to give rotation for each joint
2. Clean up cmakelists and packages.xml
