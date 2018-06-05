FROM ros:kinetic
MAINTAINER Nobuyuki Matsui <nobuyuki.matsui@gmail.com>

RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh

RUN apt update && apt upgrade -y && \
    apt install python-pip -y

COPY . /opt/ws/src/fiware-ros-turtlesim
WORKDIR /opt/ws

RUN source /opt/ros/kinetic/setup.bash && \
    /opt/ros/kinetic/bin/catkin_make && \
    echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ws/devel/setup.bash" >> /root/.bashrc && \
    cd /opt/ws/src/fiware-ros-turtlesim && \
    pip install -r ./requirements/common.txt

RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh

CMD ["/opt/ws/src/fiware-ros-turtlesim/entrypoint.sh"]
