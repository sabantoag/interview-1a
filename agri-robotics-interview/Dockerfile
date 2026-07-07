# Ready-to-run environment for the agri-robotics interview exercises.
# Build:  docker build -t agri-robotics-interview .
# Run:    docker run -it --rm -v $(pwd):/workspace agri-robotics-interview
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
