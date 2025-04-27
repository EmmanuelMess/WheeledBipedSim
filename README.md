# Wheeled Biped Sim


<img src="./images/screenshot.png"/>

### Build

```bash
docker build . -t wheeled_biped:jazzy
```


### Run

```bash
rocker --x11 \
    --device=/dev/kfd --device=/dev/dri --group-add video \
    --volume ${PWD}:/ros2_ws/src/:rw \
    --name wheeled_biped_docker \
    wheeled_biped:jazzy

colcon build --symlink-install
source install/setup.bash

ros2 launch wheeled_biped_description simulation.launch.py
```

```
docker exec -it wheeled_biped_docker bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch wheeled_biped_control basic_control.launch.py
```
