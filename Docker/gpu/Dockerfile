FROM nvidia/cuda:12.1.1-cudnn8-devel-ubuntu20.04

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS2_DISTRO=foxy
ENV ROS1_DISTRO=noetic

###################################### user #####################################

ENV SHELL=/bin/bash \
    USER=wilbur \
    UID=1000 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV HOME=/home/${USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${UID} \
    ${USER} 

RUN echo "root:root" | chpasswd
RUN echo "${USER}:111111" | chpasswd

###################################### basic tools #####################################
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-pip \
    git \
    wget \
    vim \
    wget \
    cmake \
    xterm \
    subversion \
    libfltk1.3-dev \
    libtiff5-dev \
    netbase \
    git \
    libbullet-dev \
    libpython3-dev \
    python3-empy \
    python3-numpy \
    python3-pytest \
    python3-yaml \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    apt-utils \
    && apt-get update && apt-get install -y git \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

###################################### ROS2 ####################################

RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS2_DISTRO}-desktop \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    && rosdep init && rosdep update && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

###################################### ROS1 ######################################
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && \
    apt install -y ros-${ROS1_DISTRO}-desktop-full

##################################### setting ##########################################

RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "${USER} ALL=(ALL)  ALL" >> /etc/sudoers

USER ${USER}
WORKDIR ${HOME}
CMD ["/bin/bash"]

###################################### MOOS #####################################
RUN cd ${HOME} && svn co https://oceanai.mit.edu/svn/moos-ivp-aro/trunk moos-ivp \
    && cd moos-ivp && svn update \
    && ./build-moos.sh \
    && ./build-ivp.sh \
    && echo 'export PATH=$PATH:~/moos-ivp/bin' >> ~/.bashrc