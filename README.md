# rtab_map
rtab_map

* Make sure to uninstall any rtabmap binaries:
    ```
    sudo apt remove ros-$ROS_DISTRO-rtabmap*
    ```
* RTAB-Map ROS2 package:
    ```bash
    cd ~/ros2_ws
    git clone https://github.com/introlab/rtabmap.git src/rtabmap
    git clone https://github.com/shakimaaa/rtab_map.gitsrc/rtabmap_ros
    rosdep update && rosdep install --from-paths src --ignore-src -r -y
    export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)
    export MAKEFLAGS="-j1" # when use it at jetson nano
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

* To build with `rgbd_cameras>1` support and/or `subscribe_user_data` support:
    ```bash
    colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release -DWITH_OCTOMAP_MSGS=ON -DRTABMAP_OCTOMAP=ON
    ```