ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

ARG ROS_WS=ros_ws

RUN apt-get -qq update && apt-get -qq upgrade -y && apt-get install -y \
    curl \
    unzip

WORKDIR /nvim-app
RUN curl -LO https://github.com/neovim/neovim/releases/download/v0.10.0/nvim-linux64.tar.gz
RUN tar xzvf nvim-linux64.tar.gz && rm nvim-linux64.tar.gz
RUN ln -s /nvim-app/nvim-linux64/bin/nvim /usr/local/bin

#install binary dependencies
RUN apt-get -qq update && apt-get -qq upgrade -y && apt-get install -y \
    git \
    v4l-utils \
    python3-pip \
    software-properties-common \
    python3-colcon-common-extensions \
    python3-rosdep \
    libglew-dev \
    ros-${ROS_DISTRO}-ros-testing \
    ros-${ROS_DISTRO}-ament-clang-format \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ur \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros2-control

RUN rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV QT_X11_NO_MITSHM=1
ENV EDITOR=nano
ENV XDG_RUNTIME_DIR=/tmp


ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # [Optional] Add sudo support for the non-root user
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Cleanup
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

USER ${USERNAME}

ENV ROS_DISTRO=$ROS_DISTRO
# ENV ROS_WS_DIR=/home/${USERNAME}/${ROS_WS}
# RUN mkdir -p /home/${USERNAME}/${ROS_WS}/src

# Install Moveit2 from SRC
RUN mkdir -p /home/${USERNAME}/${ROS_WS}/src
WORKDIR /home/${USERNAME}/${ROS_WS}/src
RUN export COLCON_WS=/home/${USERNAME}/${ROS_WS}
RUN git clone -b $ROS_DISTRO https://github.com/moveit/moveit2.git
RUN for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
RUN sudo rosdep update
RUN sudo rosdep install --from-paths  /home/${USERNAME}/${ROS_WS}/src --ignore-src -y -r

WORKDIR /home/${USERNAME}/${ROS_WS}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install

RUN mkdir -p /home/${USERNAME}/${ROS_WS}/src/custom

RUN echo 'LC_NUMERIC="en_US.UTF-8"' >> /home/$USERNAME/.bashrc

# Some shortcuts
RUN  echo "alias ll='ls -al'" >> /home/$USERNAME/.bashrc
RUN  echo "alias cl=clear" >> /home/$USERNAME/.bashrc

# ROS shortcuts
RUN  echo "alias b='cd ${ROS_WS} && colcon build --symlink-install && source install/setup.bash'" >> /home/$USERNAME/.bashrc
RUN  echo "alias s='cd ${ROS_WS} && source install/setup.bash'" >> /home/$USERNAME/.bashrc
RUN  echo "alias clean='rm -r ${ROS_WS}/build ${ROS_WS}/install ${ROS_WS}/log'" >> /home/$USERNAME/.bashrc
RUN  echo "alias depc='sudo rosdep update && sudo rosdep check --from-paths ${ROS_WS}/src --ignore-src -y'" >> /home/$USERNAME/.bashrc
RUN  echo "alias depi='sudo rosdep update && sudo rosdep install --from-paths ${ROS_WS}/src --ignore-src -y'" >> /home/$USERNAME/.bashrc
