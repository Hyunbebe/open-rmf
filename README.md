
# rmf_deployment_template

### Building images
You can look into the github workflow to get an idea of how to build images:
[Github Workflow](https://github.com/open-rmf/rmf_deployment_template/blob/main/.github/workflows/docker-image.yml).

From the root directory of this repo, you can run the following commands to build the images:
-------------
### humble
#### Build rmf
```bash
# Set ROS distribution and build ROS-related Docker image
ROS_DISTRO="${ROS_DISTRO:-humble}"
docker build -f rmf/builder-rosdep.Dockerfile -t rmf_deployment_template/humble-builder-rosdep .
```
```bash
# These commands are executed on your local system to setup ROS 2 and RMF environment
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install python3-vcstool -y

# Create a directory and import RMF source code
mkdir rmf-src
vcs import rmf-src < rmf/rmf.repos
```
```bash
# Build RMF Docker image
docker build -f rmf/rmf.Dockerfile -t rmf_deployment_template/humble-rmf .
```
#### Build rmf-simulation
```bash
# Import RMF Simulation source code
mkdir rmf-simulation-src
vcs import rmf-simulation-src < rmf-simulation/rmf-simulation.repos
```
```bash
# Build RMF Simulation Docker image
docker build -f rmf-simulation/rmf-simulation.Dockerfile -t rmf_deployment_template/humble-rmf-simulation .
```
#### Build rmf-web
```bash
# Import RMF Web source code
mkdir rmf-web-src
vcs import rmf-web-src < rmf-web/rmf-web.repos
```
```bash
# Builds an image for RMF web development environment.
docker build -f rmf-web/builder-rmf-web.Dockerfile -t rmf_deployment_template/humble-builder-rmf-web .
```
```bash
# Builds an image for the RMF web dashboard.
docker build -f rmf-web/rmf-web-dashboard.Dockerfile -t rmf_deployment_template/humble-rmf-web-dashboard-local .

# Builds an image for the RMF server, part of the web service.
docker build -f rmf-web/rmf-web-rmf-server.Dockerfile -t rmf_deployment_template/humble-rmf-web-rmf-server .
```
---------------
### iron
#### Build rmf
```bash
# Set ROS distribution and build ROS-related Docker image
ROS_DISTRO="${ROS_DISTRO:-iron}"
docker build -f rmf/builder-rosdep.Dockerfile -t rmf_deployment_template/iron-builder-rosdep .
```
```bash
# These commands are executed on your local system to setup ROS 2 and RMF environment
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install python3-vcstool -y

# Create a directory and import RMF source code
mkdir rmf-src
vcs import rmf-src < rmf/rmf.repos
```
```bash
# Build RMF Docker image
docker build -f rmf/rmf.Dockerfile -t rmf_deployment_template/iron-rmf .
```
#### Build rmf-simulation
```bash
# Import RMF Simulation source code
mkdir rmf-simulation-src
vcs import rmf-simulation-src < rmf-simulation/rmf-simulation.repos
```
```bash
# Build RMF Simulation Docker image
docker build -f rmf-simulation/rmf-simulation.Dockerfile -t rmf_deployment_template/iron-rmf-simulation .
```
#### Build rmf-web
```bash
# Import RMF Web source code
mkdir rmf-web-src
vcs import rmf-web-src < rmf-web/rmf-web.repos
```
```bash
# Builds an image for RMF web development environment.
docker build -f rmf-web/builder-rmf-web.Dockerfile -t rmf_deployment_template/iron-builder-rmf-web .
```
```bash
# Builds an image for the RMF web dashboard.
docker build -f rmf-web/rmf-web-dashboard.Dockerfile -t rmf_deployment_template/iron-rmf-web-dashboard-local .

# Builds an image for the RMF server, part of the web service.
docker build -f rmf-web/rmf-web-rmf-server.Dockerfile -t rmf_deployment_template/iron-rmf-web-rmf-server .
```

### Running images

Run `rmf-api-server`
```bash
docker run --rm --network host \
-it rmf_deployment_template/humble-rmf-web-rmf-server:latest
```

Run `rmf-simulation`
```bash
docker run \
-it \
--runtime=nvidia --gpus all --rm --network host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
rmf_deployment_template/humble-rmf-simulation:latest \
bash -c "ros2 launch rmf_demos_gz_classic glhd.launch.xml \
server_uri:=ws://localhost:8000/_internal"
```

```bash
docker run \
-it \
--runtime=nvidia --gpus all --rm --network host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_DOMAIN_ID=44" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
rmf_deployment_template/humble-rmf-simulation:latest \
bash -c "ros2 launch rmf_demos_gz_classic turtlebot_world.launch.xml \
server_uri:=ws://localhost:8000/_internal"
```

```bash
docker run \
-it \
--runtime=nvidia --gpus all --rm --network host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
rmf_deployment_template/humble-rmf-simulation:latest \
bash -c "ros2 topic list"
```

Run `rmf-web-dashboard`
```bash
docker run --rm -p 3000:80 \
-it rmf_deployment_template/humble-rmf-web-dashboard-local:latest
```

# rmf_traffic_editor

Run `rmf_traffic_editor`
```bash
docker compose up rmf_traffic_editor
```

save map copy to simulation-src/rmf/rmf_demos/rmf_demos_maps/maps

build rmf-simulation again

---------------------------
Run `rmf-simulation`
```bash
docker run --network=host \
-it \
--runtime=nvidia --gpus all --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/progryu/.gazebo/models:/root/.gazebo/models" \
rmf_deployment_template/humble-rmf-simulation:latest \
bash -c "export GAZEBO_MASTER_URI=http://localhost:11355 && \
ros2 launch rmf_demos_gz_classic simulation.launch.xml map_name:=turtlebot_world headless:=false"
```
Run `rmf-common`
```bash
docker run --network=host \
-it \
--runtime=nvidia --gpus all --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/progryu/.gazebo/models:/root/.gazebo/models" \
open-rmf/rmf_deployment_template/rmf-simulation:latest \
bash -c "ros2 launch rmf_demos common.launch.xml \
config_file:=/opt/rmf/install/rmf_demos_maps/share/rmf_demos_maps/turtlebot_world/turtlebot_world.building.yaml \
viz_config_file:=/opt/rmf/install/rmf_demos/share/rmf_demos/include/turtlebot_world/turtlebot_world.rviz"
```

Run `rmf-fleep-adapter`
```bash
docker run --network=host \
-it \
--runtime=nvidia --gpus all --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/progryu/.gazebo/models:/root/.gazebo/models" \
open-rmf/rmf_deployment_template/rmf-simulation:latest \
bash -c "ros2 launch rmf_demos_fleet_adapter fleet_adapter.launch.xml \
use_sim_time:=true \
nav_graph_file:=/opt/rmf/install/rmf_demos_maps/share/rmf_demos_maps/maps/turtlebot_world/nav_graphs/0.yaml \
config_file:=/opt/rmf/install/rmf_demos/share/rmf_demos/config/turtlebot_world/turtlebot3_waffle_pi_config.yaml \
server_uri:=ws://localhost:8000/_internal"
```

```bash
docker run --network=host \
-it \
--runtime=nvidia --gpus all --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/progryu/.gazebo/models:/root/.gazebo/models" \
open-rmf/rmf_deployment_template/rmf-simulation:latest \
bash -c "ls /opt/rmf/install/rmf_demos/share/rmf_demos/config/turtlebot_world/turtlebot3_waffle_pi_config.yaml"
```
---------------------------

# Build turtlebot3

#### Build turtlebot3 image
```bash
# Build turtlebot3 Docker image
docker build -f turtlebot3.Dockerfile -t rmf_deployment_template/humble-turtlebot3 .

mkdir -p ~/.gazebo/models
```

#### Build turtlebot3 image
```bash
# Build turtlebot3 Docker image
docker build -f turtlebot3.Dockerfile -t rmf_deployment_template/iron-turtlebot3 .

mkdir -p ~/.gazebo/models
```

# Run turtlebot3

#### run
```bash
# Run turtlebot3 Docker image
docker run --network=host \
-it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/zeta/.gazebo/models:/root/.gazebo/models" \
--privileged \
--runtime=nvidia \
rmf_deployment_template/iron-turtlebot3:latest \
bash -c "export GAZEBO_MASTER_URI=http://localhost:11356 && \
export TURTLEBOT3_MODEL=waffle && \
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/iron/share/turtlebot3_gazebo/models && \
ros2 launch zeta_demos_gz zeta_tb3_simulation_launch.py headless:=False use_rviz:=False"
```
```bash
# Run turtlebot3 Docker image
docker run --network=host \
-it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_DOMAIN_ID=44" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/zeta/.gazebo/models:/root/.gazebo/models" \
--privileged \
--runtime=nvidia \
rmf_deployment_template/humble-turtlebot3:latest \
bash -c "export TURTLEBOT3_MODEL=waffle_pi && \
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py"
```
```bash
# Run turtlebot3 Docker image
docker run --network=host \
-it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/zeta/.gazebo/models:/root/.gazebo/models" \
--privileged \
--runtime=nvidia \
rmf_deployment_template/humble-turtlebot3:latest \
bash -c "ros2 topic list"
```
----------------------------
### GPS EXAMPLE

```bash
# Run turtlebot3 Docker image
docker run --network=host \
-it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_DOMAIN_ID=44" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/zeta/.gazebo/models:/root/.gazebo/models" \
--privileged \
--runtime=nvidia \
rmf_deployment_template/iron-turtlebot3:latest \
bash -c "ros2 launch nav2_gps_waypoint_follower_demo gazebo_gps_world.launch.py"
```

```bash
# Run turtlebot3 Docker image
docker run --network=host \
-it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_DOMAIN_ID=32" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/zeta/.gazebo/models:/root/.gazebo/models" \
--privileged \
--runtime=nvidia \
rmf_deployment_template/iron-turtlebot3:latest \
bash -c "ros2 launch nav2_gps_waypoint_follower_demo navsat.launch.py"
```
```bash
# Run turtlebot3 Docker image
docker run --network=host \
-it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_DOMAIN_ID=32" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/zeta/.gazebo/models:/root/.gazebo/models" \
--privileged \
--runtime=nvidia \
rmf_deployment_template/iron-turtlebot3:latest \
bash -c "ros2 run tf2_ros tf2_echo map base_link"
```


```bash
# Run turtlebot3 Docker image
docker run --network=host \
-it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_DOMAIN_ID=32" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/zeta/.gazebo/models:/root/.gazebo/models" \
--privileged \
--runtime=nvidia \
rmf_deployment_template/iron-turtlebot3:latest \
bash -c "ros2 topic list"
```

```bash
# Run turtlebot3 Docker image
docker run --network=host \
-it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_DOMAIN_ID=32" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/zeta/.gazebo/models:/root/.gazebo/models" \
--privileged \
--runtime=nvidia \
rmf_deployment_template/iron-turtlebot3:latest \
bash -c "ros2 launch nav2_gps_waypoint_follower_demo mapviz.launch.py"
```

```bash
# Run turtlebot3 Docker image
docker run --network=host \
-it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_DOMAIN_ID=44" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/zeta/.gazebo/models:/root/.gazebo/models" \
--privileged \
--runtime=nvidia \
rmf_deployment_template/iron-turtlebot3:latest \
bash -c "ros2 launch nav2_gps_waypoint_follower_demo gps_waypoint_follower.launch.py use_rviz:=True use_mapviz:=True"
```

```bash
# Run turtlebot3 Docker image
docker run --network=host \
-it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_DOMAIN_ID=32" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/zeta/.gazebo/models:/root/.gazebo/models" \
--privileged \
--runtime=nvidia \
rmf_deployment_template/iron-turtlebot3:latest \
bash -c "ls /opt/turtlebot3/install/nav2_gps_waypoint_follower_demo/share/nav2_gps_waypoint_follower_demo/worlds"
```
--------------------------------
```bash
docker run --network=host \
-it \
--runtime=nvidia --gpus all --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/progryu/.gazebo/models:/root/.gazebo/models" \
open-rmf/rmf_deployment_template/rmf-simulation:latest \
bash -c "ros2 topic echo /clicked_point"
```


#### Build ecobot_fleep
```bash
# Build RMF Simulation Docker image
docker build -f Dockerfile -t open-rmf/rmf_deployment_template/rmf-simulation-ecobot .
```

#### Run ecobot_fleep
Run `rmf-simulation`
```bash
docker run --network=host \
-it \
--runtime=nvidia --gpus all --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
open-rmf/rmf_deployment_template/rmf-simulation-ecobot:latest \
bash -c "ros2 launch rmf_demos office.launch.xml run_fleet_adapters:=0"
```

```bash
docker run --network=host \
-it \
--runtime=nvidia --gpus all --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
open-rmf/rmf_deployment_template/rmf-simulation-ecobot:latest \
bash -c "ros2 run fleet_adapter_ecobot fleet_adapter_ecobot \
    -c src/fleet_adapter_ecobot/configs/robot_config.yaml \
    -n install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml \
    -s "http://localhost:8000/_internal" \
    -tf src/fleet_adapter_ecobot/configs/test_api_config.yaml"
```

```bash
docker run --network=host \
-it \
--runtime=nvidia --gpus all --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
open-rmf/rmf_deployment_template/rmf-simulation-ecobot:latest \
bash -c "ros2 run rmf_demos_tasks dispatch_patrol -p pantry"
```

### Examples

```bash
docker build -f irontest.Dockerfile -t irontest/roslibpy .

docker build -f humbletest.Dockerfile -t humbletest/roslibpy .
```

```bash
docker run --network=host \
-it \
--runtime=nvidia --gpus all --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_DOMAIN_ID=44" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="./gps_test.py:/gps_test.py" \
humbletest/roslibpy:latest \
bash -c "python3 gps_test.py"
```
