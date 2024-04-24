FROM ros:noetic-perception

ENV CATKIN_WS=/root/catkin_ws \
    GNSS_SI_ROOT=/root/catkin_ws/src/gnss-stereo-inertial-fusion/ 

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y \
    git \
    python3-pip \
    libgeographic-dev \
    # libpython2.7-dev \
    libglew-dev && \
    rm -rf /var/lib/apt/lists/* && \
    mkdir src && cd src && \
    git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    git checkout 86eb4975fc4fc8b5d92148c2e370045ae9bf9f5d && \
    mkdir build && \
    cd build && \
    cmake .. && \
    cmake --build .

COPY ./ $GNSS_SI_ROOT

# Build it
#WORKDIR $CATKIN_WS
#COPY ./scripts/ $CATKIN_WS
WORKDIR $GNSS_SI_ROOT
RUN ["/bin/bash", "-c", "chmod +x modify_entrypoint.sh && sync && ./modify_entrypoint.sh"]
RUN ["/bin/bash", "-c", "chmod +x build.sh && chmod +x build_ros.sh && sync && source /opt/ros/$ROS_DISTRO/setup.bash && export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$GNSS_SI_ROOT/Examples/ROS && ./build.sh && ./build_ros.sh"]
