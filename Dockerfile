FROM nvidia/cuda:8.0-cudnn6-devel-ubuntu16.04

ENV TF_MIN_GPU_MULTIPROCESSOR_COUNT 6
ARG proxy
ENV http_proxy $proxy
ENV https_proxy $proxy

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' && \
  apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
  apt-get update && \
  apt-get install -y \
    python-pip \
    ros-kinetic-desktop-full && \
  apt-get clean && \
  rm -rf \
          /tmp/* \
          /var/lib/apt/lists/* \
          /var/tmp/*

# Install dbw_mkz
RUN apt-get update && apt-get install -y wget curl vim
COPY dbw_mkz/* /tmp/
RUN cd /tmp && ./sdk_update.bash && \
  rm /tmp/sdk_update.bash /tmp/sdk_install.bash

# Initialize rosdep
RUN rosdep init

# Install python dependencies
ARG PYTHON_DEPS=requirements.txt
COPY $PYTHON_DEPS /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt && \
  rm /tmp/requirements.txt

# Create udacity user
ARG C_UID=1000
ARG C_GID=999
RUN groupadd -g $C_GID udacity
RUN useradd --create-home -u $C_UID -g $C_GID udacity

#COPY bashrc_udacity /home/udacity/.bashrc_udacity
#RUN echo "source /home/udacity/.bashrc_udacity" >> /home/udacity/.bashrc && \
#  echo "source /opt/ros/kinetic/setup.bash" >> /home/udacity/.bashrc

VOLUME /udacity
RUN chown udacity:udacity /udacity

USER udacity:udacity
RUN rosdep update
WORKDIR /udacity
