# ROS2 base image
FROM ros:jazzy AS base
# @sha256:f39d671d65cb1e7fb7add336579d6e9a3ac0d35ffcc822f598977a580ead9366

# Install dependencies with apt
RUN apt update && \
  DEBIAN_FRONTEND=noninteractive apt install -y keyboard-configuration && \
  apt install -y git \
  apt-utils \
  software-properties-common \
  bash-completion \
  vim nano tmux htop \
  gdb mold \
  desktop-file-utils \
  ros-dev-tools ros-jazzy-foxglove-bridge\
  python3-colcon-common-extensions \
  python3-pip \
  python3-venv \
  p7zip-full \
  libpcap-dev \
  iproute2 \
  gnuplot \
  libboost-all-dev \
  libncurses5-dev libncursesw5-dev \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-foxglove-compressed-video-transport \
  lm-sensors smartmontools\
  ros-jazzy-rqt ros-jazzy-rqt-common-plugins \
  ros-jazzy-ament-clang-format ros-jazzy-ament-cmake-clang-format portaudio19-dev pipewire-audio-client-libraries \
  ros-jazzy-image-transport-plugins ros-jazzy-image-common \
  ffmpeg intel-media-va-driver-non-free libmfx1 

RUN pip install --break-system-packages \
    mypy \
    colcon-clean

FROM base AS dev

# Dev container arguments
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN usermod -md /home/$USERNAME -l $USERNAME ubuntu

# make sudo available
RUN echo "$USERNAME" ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/"$USERNAME" && \
  chmod 0440 /etc/sudoers.d/"$USERNAME"
RUN chown -R ${USERNAME}:${GID} /home/${USERNAME}

# run this one without password
RUN echo "$USERNAME ALL=(ALL) NOPASSWD: /usr/sbin/smartctl" >> /etc/sudoers

# Set the user and source entrypoint in the user's .bashrc file
RUN mkdir /home/$USERNAME/.bash_aliases && echo "source <(cat /home/$USERNAME/.bash_aliases/*.sh)" >> /home/"$USERNAME"/.bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /home/$USERNAME/vcu_ws/install/setup.bash" >> /home/"$USERNAME"/.bashrc
RUN echo "export COLCON_DEFAULTS_FILE=/home/devuser/vcu_ws/src/configs/colcon_config.yaml" >> /home/"$USERNAME"/.bashrc
RUN chsh -s /bin/bash $USERNAME
ENV SHELL=/bin/bash
USER $USERNAME

# Install colcon mixins
RUN cd /home/"$USERNAME"/ && colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml &&\
  colcon mixin update default

CMD ["/bin/bash"]

