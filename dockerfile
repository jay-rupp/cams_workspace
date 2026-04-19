FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    joystick \
    python3-pygame \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]
