ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}

# Install some basic stuff
RUN apt-get update && apt-get install -y \
    tree

# Install UV and add it to PATH
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:$PATH"

# Create a workspace directory
WORKDIR /ros2_ws

# Copy the main colcon-uv package (excluding examples to avoid duplicates)
COPY colcon_uv /ros2_ws/tools/colcon_uv

# Create a COLCON_IGNORE file to avoid building the library
RUN touch /ros2_ws/tools/COLCON_IGNORE

# Install colcon-uv
RUN cd /ros2_ws/tools/colcon_uv && uv pip install --system --break-system-packages .

# Copy examples folder to workspace level for colcon discovery
COPY tests/examples /ros2_ws/src/examples

# Copy and make scripts executable
COPY tests/docker/entrypoint.sh /ros2_ws/entrypoint.sh
COPY tests/docker/test_pingpong.sh /ros2_ws/test_pingpong.sh
COPY tests/docker/test_isolation.sh /ros2_ws/test_isolation.sh
RUN chmod +x /ros2_ws/entrypoint.sh /ros2_ws/test_pingpong.sh /ros2_ws/test_isolation.sh

# Build both example packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /ros2_ws && \
    colcon build --packages-select uv_ament_python_example uv_ament_cmake_example --symlink-install

# Source the workspace in .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc 