FROM ros:noetic-ros-core
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y python3-pip&& \
    apt-get autoremove --purge && rm -rf /var/cache/apt/*

WORKDIR /app
COPY ./vehicle /app
COPY ./msgs /msgs

RUN pip3 install rospkg
RUN ./setup.sh

CMD [ "./docker-entrypoint.sh" ]
