FROM ros:melodic-ros-core
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-melodic-rosserial-python build-essential && \
    apt-get autoremove --purge && rm -rf /var/cache/apt/*

WORKDIR /app
COPY ./mbed /app
COPY ./msgs /msgs
RUN ./setup.sh

CMD [ "./docker-entrypoint.sh" ]
