FROM ros:humble-ros-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y software-properties-common \
    curl \
    git

RUN git clone https://github.com/KYabuuchi/MapCleaner_Unofficial.git /MapCleaner/src && \
    cd /MapCleaner/src && \
    git checkout develop/ros2 && \
    cd /MapCleaner && \
    rosdep update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    . /opt/ros/humble/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY ./ENTRYPOINT.sh /ENTRYPOINT.sh
RUN chmod +x /ENTRYPOINT.sh
ENTRYPOINT ["/ENTRYPOINT.sh"]