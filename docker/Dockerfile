FROM ros:jazzy-ros-base-noble

# Path to the default ROS workspace
ENV HOME=/home/ros
ENV ROS_WS=${HOME}/workspace/ros

#============= Install extra software packages =============#

# Install Redland, Raptor2, libmongoc, spdlog, fmt, gtest
USER root
RUN apt update && \
    apt install -y \
        gdb g++ clang cmake make doxygen \
        libeigen3-dev \
        libspdlog-dev \
        libraptor2-dev \
        libboost-all-dev \
        libboost-dev \
        libboost-python-dev \
        librdf0 \
        librdf0-dev \
        libmongoc-1.0-0 \
        libmongoc-dev \
        libgtest-dev \
        libfmt-dev \
        software-properties-common

# Install SWI Prolog
# Source: https://www.swi-prolog.org/build/PPA.html
USER root
RUN apt update && \
    apt install -y swi-prolog*
ENV SWI_HOME_DIR=/usr/lib/swi-prolog
ENV LD_LIBRARY_PATH=/usr/lib/swi-prolog/lib/x86_64-linux

# Install Knowrob
# Source: https://github.com/knowrob/knowrob
WORKDIR ${ROS_WS}/src
RUN git clone https://github.com/knowrob/knowrob.git -b dev
# RUN git clone https://github.com/knowrob/knowrob_ros.git -b ros2
# Fix KnowRob Commit
WORKDIR ${ROS_WS}/src/knowrob
RUN git fetch && \
    git checkout 6cf51f3210509f4e3db362c93fe753319b389cac
# Fix KnowRob ROS Commit
# WORKDIR ${ROS_WS}/src/knowrob_ros
# RUN git fetch && \
#    git checkout 6cf51f3210509f4e3db362c93fe753319b389cac
# Source and build workspace
RUN colcon build --merge-install --symlink-install

RUN echo "source ${ROS_WS}/install/setup.bash" >> ${HOME}/.bashrc
