# kinect2_tracker
A working ROS wrapper for the KinectOne (v2) using libfreenect2

## install 

- [install libfreenect2](https://github.com/OpenKinect/libfreenect2/)
  - Make sure to install all the optional stuff, including OpenCL and **OpenNI2**
  - When you build the library, do not follow the instructions there, instead run
  ```bash
  mkdir build && cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/
  make
  sudo make install
  ```
- [Download NiTE2](http://openni.ru/files/nite/index.html) and put it in `~/package_ws/NiTE-Linux-x64-2.2/`
  - Or you can put it in some other random places, but you need to modify `CMakeList.txt` and `setup_nite.bash`
- `source setup_nite.bash`

To run the program the launch file needs to be used

## Run

```bash
roslaunch kinect2_tracker tracker.launch
```

## API

### Published

- `/people_skeleton` : `kinect2_tracker::user_IDs`, id array of the tracked people
- `/people_points`: `kinect2_tracker::user_points`, center of mass for each person
- `/people_points_viz`: `visualization_msgs::Marker`, people points to show in `rviz`
- `tf` transforms for the human skeletons 
- Kinect RGB, depth and infrad images

### Params

- `tf_prefix`: The prefirx when publishing tf
- `relative_frame`: The base frame of the Kinect observations
