FROM ros:noetic

RUN apt update 
RUN apt install rviz -y
RUN apt install ros-noetic-moveit -y 
RUN apt install curl -y
RUN apt install apt-utils -y 
RUN apt install python3-pip -y 
RUN apt install python-is-python3 -y 
RUN apt install net-tools -y 
RUN apt install nmap -y 
RUN apt install xauth -y
RUN apt install ros-noetic-robot-state-publisher -y
RUN apt install git -y
RUN apt install ros-noetic-usb-cam -y
RUN apt install ros-noetic-image-view -y
RUN apt install python3-catkin-tools -y
RUN apt install libxmlrpc-core-c3-dev -y

RUN pip install numpy
RUN pip install -U catkin_tools
RUN pip install ur_rtde
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

ENTRYPOINT [ "sleep", "infinity"]