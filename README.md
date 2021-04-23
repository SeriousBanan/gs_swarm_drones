# gs_swarm_drones

A repository with the source code for launching a swarm of Pioneer-Max drones performing various tasks.

## Requirements

-   ROS Noetic (installation: <http://wiki.ros.org/noetic/Installation>)
-   Python 3.8 or newer (Download: <https://www.python.org/downloads/>)

## Installation

1. Clone geoscan_pioneer_max repository.

    ```bash
    git clone https://github.com/geoscan/geoscan_pioneer_max.git
    cd geoscan_pioneer_max
    git submodule update --init
    ```

2. Go to Geoscan ROS Workspace.

    ```bash
    cd geoscan_ws/src
    ```

3. Clone gs_swarm_drones repository.

    ```bash
    git clone https://github.com/SeriousBanan/gs_swarm_drones.git
    ```

4. Build code in workspace.

    ```bash
    cd <path to geoscan_ws>
    source /opt/ros/noetic/setup.bash
    catkin_make
    ```

<!-- ## Launching -->

<!-- TODO add launching -->
<!-- 2. source workspace setup.bash -->
<!-- 3. launching launch file -->
