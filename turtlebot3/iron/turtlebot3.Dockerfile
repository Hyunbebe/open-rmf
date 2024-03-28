ARG BUILDER_NS="rmf_deployment_template"
ARG TAG="latest"
FROM $BUILDER_NS/iron-rmf:$TAG

ARG NETRC

SHELL ["bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN apt update && apt install -y \
    ros-iron-rviz2 \
    ros-iron-navigation2 \
    ros-iron-nav2-bringup \
    ros-iron-slam-toolbox \
    ros-iron-robot-localization \
    ros-iron-mapviz\
    ros-iron-mapviz-plugins \
    ros-iron-tile-map \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/$ROS_DISTRO/setup.bash && apt update && apt install -y \
    ros-iron-turtlebot3-gazebo \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/turtlebot3

COPY zeta_demos_gz src/zeta_demos_gz
COPY navigation2_tutorials src/navigation2_tutorials

RUN rosdep update --rosdistro $ROS_DISTRO

RUN apt update && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


RUN sed -i '$isource "/opt/turtlebot3/install/setup.bash"' /ros_entrypoint.sh

RUN pip install roslibpy

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
