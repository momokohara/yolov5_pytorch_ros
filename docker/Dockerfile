FROM nvidia/cuda:11.2.2-cudnn8-runtime-ubuntu20.04

USER root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && \
    apt upgrade -y && \
    apt install -y \
        git \
        curl \
        wget \
        vim \
        build-essential \
        libgeos-dev \
        libopencv-dev \
        python3 \
        python3-distutils \
    && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# install pytorch
RUN curl -kL https://bootstrap.pypa.io/get-pip.py | python3 
ENV PATH $PATH:/home/${DOCKER_USER}/.local/bin
RUN pip3 install --no-cache-dir torch==1.7.0+cu110 torchvision==0.8.1+cu110 torchaudio==0.7.0 -f https://download.pytorch.org/whl/torch_stable.html

# install YOLO
WORKDIR /root
RUN apt update && \
    apt install -y \
        python3-dev \
    && \
    curl https://raw.githubusercontent.com/tiger0421/yolov5_pytorch_ros/use_gpu/requirements.txt > requirements.txt && \
    python3 -m pip install --no-cache-dir -r requirements.txt && \
    rm requirements.txt && \
    apt remove --purge -y \
        python3-dev \
    && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN ln -s /usr/bin/python3 /usr/bin/python 

# install ROS noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt update && \
    apt install -y \
        ros-noetic-desktop \
    && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo source /opt/ros/noetic/setup.bash >> /root/.bashrc
