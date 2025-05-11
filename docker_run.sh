# Run the Docker container
docker run -it --rm \
  --privileged \
  --device=/dev/sensors/dynamixel \
  -v /dev:/dev \
  dynamixel_control:latest \
  bash 