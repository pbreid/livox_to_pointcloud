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
    git clone https://github.com/pbreid/livox_to_pointcloud.git
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

You can run the converter script directly from the command line, specifying the input and output topics.

```bash
python3 src/pointConverter/test.py --input_topic /custom_pointcloud_laser1 --output_topic /pointcloud2_laser1 --node_name laser1_converter_node
python3 src/pointConverter/test.py --input_topic /custom_pointcloud_laser2 --output_topic /pointcloud2_laser2 --node_name laser2_converter_node
```

Command-Line Arguments
--input_topic: The topic to subscribe to for the custom point cloud message (CustomMsg).
--output_topic: The topic to publish the PointCloud2 message.


Running from a ROS Launch File
You can use a ROS launch file to start multiple instances of the converter node with different topics.

Create a launch file named multi_laser.launch in your launch directory:
```
<launch>
  <node pkg="pointConverter" type="test.py" name="laser1_converter" output="screen" args="--input_topic /custom_pointcloud_laser1 --output_topic /pointcloud2_laser1 --node_name laser1_converter_node" />
  <node pkg="pointConverter" type="test.py" name="laser2_converter" output="screen" args="--input_topic /custom_pointcloud_laser2 --output_topic /pointcloud2_laser2 --node_name laser2_converter_node" />
  <!-- Add more nodes as needed -->
</launch>
```

Run the launch file:

```
roslaunch pointConverter multi_laser.launch
```