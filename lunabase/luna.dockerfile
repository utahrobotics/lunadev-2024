FROM ubuntu as luna

LABEL org.opencontainers.image.source=https://github.com/utahrobotics/lunadev-2024
LABEL org.opencontainers.image.description="An image ready for ROS 2 development"

# Configure timezone
ENV TZ=America/Denver
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Enable Ubuntu Universe Repository
RUN apt-get update && \
    apt-get install software-properties-common -y --no-install-recommends && \
    add-apt-repository universe

# Install installation dependencies: curl, pip
RUN apt-get install curl pip -y --no-install-recommends

# Install dev tools: git, usbutils
RUN apt-get install git usbutils -y --no-install-recommends

# Add ROS 2 GPG key then add the repository to sources list
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y

# Install ROS 2
RUN apt-get install ros-humble-ros-base python3-argcomplete -y --no-install-recommends

# Install hidapi dependencies
RUN apt-get install python-dev libusb-1.0-0-dev libudev-dev -y --no-install-recommends

# Upgrade setuptools, then install pip packages not present in rosdep, and rosdep itself
RUN pip install --upgrade setuptools && \
    pip install pyvesc hidapi rosdep

# Init and update rosdep
RUN rosdep init && rosdep update

# Install build tools: colcon, build-essential
RUN apt-get install python3-colcon-common-extensions build-essential -y --no-install-recommends

# install realsense 2 ROS
RUN apt-get install ros-humble-realsense2-camera -y --no-install-recommends

# Install nav2 components as separate layers (to make it easier to upload and cache)
RUN apt-get install ros-humble-navigation2 -y --no-install-recommends
RUN apt-get install ros-humble-nav2-bringup '~ros-humble-turtlebot3-.*' -y --no-install-recommends

# Install rviz2
RUN apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-rviz-visual-tools

COPY base_bashrc_append.sh /bashrc_append
RUN ["/bin/bash", "-c", "cat /bashrc_append >> /root/.bashrc && rm /bashrc_append"]
