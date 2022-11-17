FROM wangyan94maomao/robot_learning:latest
LABEL maintainer="Wang, Yan" email="wangyan@frtech.fr"

RUN /bin/bash -c "cd ~/ros_ws/src \
    && git clone https://github.com/issaiass/realsense2_description.git realsense2_description"

RUN /bin/bash -c "cd ~/ros_ws/src \
    && git clone https://github.com/issaiass/realsense_gazebo_plugin.git realsense_gazebo_plugin"

# Compiling ros workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd ~/ros_ws/ \
    && rm -rf build \
    && catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so -DPYTHON_VERSION=3"
