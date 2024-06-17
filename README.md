# Custom Point Cloud to PointCloud2 Converter

This project contains a ROS node that converts custom point cloud messages (`CustomMsg`) to `sensor_msgs/PointCloud2` messages. This is particularly useful for visualizing custom point cloud data in tools like Foxglove Studio.

## Requirements

- ROS Noetic
- Python 3
- `custom_msgs` package with `CustomMsg` and `CustomPoint` message definitions

## Installation

1. Clone this repository into your catkin workspace:

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/your-repo/livox_to_pointcloud.git
    ```

2. Build the workspace:

    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

3. Source the workspace:

    ```bash
    source devel/setup.bash
    ```

## Usage

### Running Directly from Python

You can run the converter script directly from the command line, specifying the input and output topics, and a unique node name.

```bash
python3 src/livox_to_pointcloud/convert_points.py --input_topic /custom_pointcloud_laser1 --output_topic /pointcloud2_laser1 --node_name laser1_converter_node
python3 src/livox_to_pointcloud/convert_points.py --input_topic /custom_pointcloud_laser2 --output_topic /pointcloud2_laser2 --node_name laser2_converter_node
```

OR

```bash
roslaunch livox_to_pointcloud avia.launch
roslaunch livox_to_pointcloud mid70.launch
```