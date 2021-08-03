FROM ros:noetic-ros-core

WORKDIR /app
COPY ./joystick /app

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y python3-pip libusb-1.0-0-dev libudev-dev curl ros-noetic-joy && \
    rm -rf /var/cache/apt/*

RUN pip3 install rospkg
RUN ./setup.sh

CMD [ "./docker-entrypoint.sh" ]
