# gs_swarm_drones

A repository with the source code for launching a swarm of Pioneer-Max drones performing various tasks.

## Requirements

-   ROS Noetic (installation: <http://wiki.ros.org/noetic/Installation>)
-   Python 3.8 (Download: <https://www.python.org/downloads/>)
-   Operator application on PC (<https://github.com/SeriousBanan/gs_swarm_drones_operator>)

## Installation on drone

1. Clone geoscan_pioneer_max repository if Pioneer-Max drone hasn't got it.

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

5. Install required python modules.

    ```bash
    python3 -m pip install -r <path to gs_swarm_drones>/requirements.txt
    ```

## Preparing to launch

1. Go to folder with gs_swarm_drones repository.

    ```bash
    cd <path to gs_swarm_drones>
    ```

2. Generate field file.

    ```bash
    python3 -m generate_field.py <field with> <field_height> <vertex_width> <vertex_height>
    ```

    It will generate json file `field.json` with field definition of vertexes (ids and positions) and edges between vertexes.

3. Connect drones in one Network.

    If NetworkManager disabled enable it by bash script.

    ```bash
    bash bash/enable_NetworkManager.bash
    ```

    After that drone stop his hotspot.

    Connect to WiFi:

    ```bash
    nmcli device wifi connect "<WiFi SSID>" password "<WiFi password>" name "<give name to connection>"
    ```

    After that check the drone ip by ifconfig utile.

4. Fill json configuration file.

    You need to remove `...` and fill fields in configuration.json file:

    - **drone id**: int value from _0_ to _count of all drones_.

        For example if you have _3_ drones you can set here _0_, _1_ or _2_.

    - **drones count**: int value of amount of drones.
    - **drones initial positions**: dict where keys - drone's ids, values - ids of vertexes where drone start it program (from field.json file).

        For example it can be:

        ```json
        {
            "0": 0,
            "1": 6
        }
        ```

    - **drone flight altitude**: float value of altitude on which drone fill flight.
    - **addresses**: dict where keys - drones's ids or operator, values - dict with ips and ports.

        For example:

        ```json
        "0": {
            "ip": "192.168.1.19",
            "port": 10000
        }
        ```

        This information used to communication between drones and operator.

    Example of full filled configuration file:

    ```json
    {
        "drone id": 0,
        "drones count": 2,
        "drones initial positions": {
            "0": 6,
            "1": 0
        },
        "drone flight altitude": 1,
        "addresses": {
            "operator": {
                "ip": "192.168.1.2",
                "port": 10000
            },
            "0": {
                "ip": "192.168.1.19",
                "port": 10000
            },
            "1": {
                "ip": "192.168.1.18",
                "port": 10000
            }
        },
        "image file path": "received_image.png"
    }
    ```

5. Install the switch on the remote control to the manual control mode.

6. Restart pioneer by terminal:

    ```bash
    rospioneer restart
    ```

7. Set drones to it's start positions.

8. Move the switch on the console to the software mode.

## Launching drones

Launch program:

```bash
roslaunch gs_swarm_drones gs_swarm_drones.launch
```

After that drones will start waiting from operator command to start.

Click to button with label `start` and drones will takeoff.

When one drone found tank image from it's camera fill appear at right bottom corner.
