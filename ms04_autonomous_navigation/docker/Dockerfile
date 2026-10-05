# 1. Base Image
FROM osrf/ros:jazzy-desktop-full
ENV DEBIAN_FRONTEND=noninteractive

# 2. Create a clean workspace shell layout inside the container
WORKDIR /test_ws/src/ms04_autonomous_navigation

# 3. Copy your local repository package files straight into this folder
COPY . .

# 4. Clean up any accidental host build artifacts brought over by Git
RUN rm -rf build install log docker

# 5. Run rosdep relative to the container workspace root
WORKDIR /test_ws
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro jazzy

# 6. Build the package via colcon
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# 7. Setup automatic sourcing
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /test_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]

