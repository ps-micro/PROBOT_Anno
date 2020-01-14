FROM ros:kinetic-ros-base
ENV ROS_HOSTNAME=127.0.0.1
ENV ROS_MASTER_URI=http://127.0.0.1:11311

# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    locate \
    bash-completion \
    ros-kinetic-ros-tutorials \
    ros-kinetic-common-tutorials \
    ros-kinetic-moveit-* \
    ros-kinetic-industrial-* \
    ros-kinetic-ros-control \
    ros-kinetic-ros-controllers \
    ros-kinetic-trac-ik-kinematics-plugin \
    ros-kinetic-usb-cam \    
    libblas-dev \
    liblapack-dev \
    && rm -rf /var/lib/apt/lists/
