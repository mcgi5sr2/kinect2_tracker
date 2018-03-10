# kinect2_tracker
A working ROS wrapper for the KinectOne (v2) using libfreenect2

## install 

- [install libfreenect2](https://github.com/OpenKinect/libfreenect2/)
- [Install NiTE2](http://openni.ru/files/nite/index.html) and put it in `~/package_ws/NiTE-Linux-x64-2.2/`
- `source setup_note.bash`
- The libOpenni2.so library needs to be copied to your /usr/lib dir

To run the program the launch file needs to be used

## Run

```bash
roslaunch kinect2_tracker tracker.launch
```

## API

### Published

 - `/people_skeleton` : `kinect2_tracker::user_IDs`, id array of the tracked people
 - `/people_points`: `kinect2_tracker::user_points`, person centers
 - `/people_points_viz`: `geometry_msgs::PointStamped`, one point to show in `rviz`
- `tf` transforms from 

### Params

- `tf_prefix`: The prefirx when publishing tf
- `relative_frame`: The base frame of the Kinect observations