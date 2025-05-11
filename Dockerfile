# Use an ARM64 ROS2 base image (adjust distro as needed)
FROM arm64v8/ros:jazzy
ENV ROS_DISTRO=jazzy

# Install necessary tools and dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    python3-colcon-common-extensions \
    python3-venv \
    python3-pip \
 && rm -rf /var/lib/apt/lists/*

# Set workspace directory
WORKDIR /app

# Create and activate virtual environment for Dynamixel SDK
RUN python3 -m venv /app/venv
ENV PATH="/app/venv/bin:$PATH"

# Install Dynamixel SDK and dependencies
RUN pip install dynamixel-sdk pyserial

# Copy the entrypoint script and ensure it's executable
COPY docker_entrypoint.sh /app/
RUN chmod +x /app/docker_entrypoint.sh

# Set the entrypoint and default command
ENTRYPOINT ["/app/docker_entrypoint.sh"]
CMD ["bash"]