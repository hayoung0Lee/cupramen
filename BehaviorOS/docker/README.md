# Docker Development Environment

ROS2 Humble + BehaviorOS development environment for ROS integration development and examples.

## Purpose

While BehaviorOS is currently a pure Python library, this Docker environment is intended for:

- **ROS2 integration development**: BehaviorOS will offer a ROS layer to make it compatible with ROS2
- **Example demonstrations**: Examples will show how to use BehaviorOS in a ROS environment
- **Consistent testing**: Pre-configured ROS2 Humble setup for developing and testing the ROS integration
- **Isolated environment**: Test ROS-specific features without affecting your host system

The `behavioros` package is mounted as a volume, so you can edit code on your host machine and immediately test changes inside the container.

## Usage

### Initial Setup (One-time)
```bash
cd docker
docker compose build
```

### Running the Container

#### Option 1: One-off Session
```bash
docker compose run --rm behavioros
```
Container is automatically removed when you exit.

#### Option 2: Background Service (Recommended)
```bash
# Start container in background (once)
docker compose up -d

# Attach to container (can be done multiple times)
docker compose exec behavioros bash

# Open another terminal and attach again
docker compose exec behavioros bash

# Stop and remove container when done
docker compose down
```

### Inside the Container
```bash
# Check ROS2 is working
ros2 topic list

# Run examples (when available)
# python3 /opt/behavioros/examples/ros_example.py

# Build ROS2 workspace (when ROS packages are added)
# cd /ros2_ws && colcon build
```