FROM ubuntu as base

LABEL org.opencontainers.image.source=https://github.com/utahrobotics/lunadev-2024
LABEL org.opencontainers.image.description="An image ready for ROS 2 development"

# Configure timezone
ENV TZ=America/Denver
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Enable Ubuntu Universe Repository
RUN apt-get update && \
    apt-get install software-properties-common -y --no-install-recommends && \
    add-apt-repository universe
RUN apt-get install curl pip -y --no-install-recommends

# Add ROS 2 GPG key then add the repository to sources list
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y

# Install ROS 2
RUN apt-get install ros-humble-ros-base python3-argcomplete -y --no-install-recommends

# Install GPIO and VESC stuff, and rosdep
RUN pip install Jetson.GPIO pyvesc rosdep
# Install colcon, build essentials, and usbutils
RUN apt-get install python3-colcon-common-extensions build-essential usbutils -y --no-install-recommends
# Update rosdep
RUN rosdep init && rosdep update

# install realsense 2 ROS
RUN apt-get install ros-humble-realsense2-camera -y --no-install-recommends
# Install nav2 components as separate layers (to make it easier to upload and cache)
RUN apt-get install ros-humble-navigation2 -y --no-install-recommends
RUN apt-get install ros-humble-nav2-bringup '~ros-humble-turtlebot3-.*' -y --no-install-recommends

COPY base_bashrc_append.sh /bashrc_append
# COPY findusbdev.sh /root
# RUN chmod +x /root/findusbdev.sh
# Append bashrc
RUN ["/bin/bash", "-c", "cat /bashrc_append >> /root/.bashrc && rm /bashrc_append"]

FROM base as lunabase

LABEL org.opencontainers.image.source=https://github.com/utahrobotics/lunadev-2024
LABEL org.opencontainers.image.description="A development image that contains NoMachine, git, and github"

EXPOSE 4000

# Install nano and git
RUN apt-get install nano git -y --no-install-recommends

# Install NoMachine
RUN curl -o nx.deb https://download.nomachine.com/download/8.4/Linux/nomachine_8.4.2_1_amd64.deb && \
    dpkg -i nx.deb && \
    rm nx.deb

# Install rviz2
RUN apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-rviz-visual-tools

COPY lunabase_bashrc_append.sh /bashrc_append
RUN ["/bin/bash", "-c", "cat /bashrc_append >> /root/.bashrc && rm /bashrc_append"]
