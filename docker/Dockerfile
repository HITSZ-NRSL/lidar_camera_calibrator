FROM ros:melodic-perception
RUN apt-get update && apt-get install -y x11-apps git vim libcairo2-dev libcgal-dev libopencv-dev python3-opencv\
    libeigen3-dev liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev \ 
    qt5-qmake qt5-default qtbase5-dev qtbase5-private-dev libqglviewer-dev-qt5 libsuitesparse-dev libcxsparse3 libcholmod3 

ARG user=calibrator
ARG home=/home/$user
RUN groupadd -g 1000 $user
RUN useradd -d $home -s /bin/bash -m $user -u 1000 -g 1000 \
        && adduser $user sudo
WORKDIR $home

COPY ext .
RUN cd ceres-solver && cmake . && make -j3 && make install
RUN cd g2o && cmake . && make -j3 && make install
# see issue https://github.com/strasdat/Sophus/issues/214 for ubuntu 20.04
RUN cd Sophus && cmake . && make -j3 && make install

ENV ROS_WS $home/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
COPY . $ROS_WS/src/lidar_camera_calibrator
RUN . /opt/ros/melodic/setup.sh \
    && catkin_make


# fix vtk pcl interaction https://stackoverflow.com/a/75687353
RUN ln -s /usr/bin/vtk7 /usr/bin/vtk

RUN chown -R $user:$user $home
USER $user
ENV HOME $home

# necessary for libg2o_core.so to be found after linking
RUN echo "export LD_LIBRARY_PATH=/usr/local/lib" >> ~/.bashrc
RUN echo "export XDG_RUNTIME_DIR=$home/xdg_runtime_dir" >> ~/.bashrc

# xdg
RUN mkdir $home/xdg_runtime_dir
