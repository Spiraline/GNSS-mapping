# GNSS mapping

Make points map with GNSS

## Usage

**[Requirement]**

- ROS
- `setup.bash` in ROS should be already sourced
(ex: `source /opt/ros/melodic/setup.bash`)

**[Input topic]**

- pointcloud (`sensor_msgs::PointCloud2` type)
- gnss (`geometry_msgs::PoseStamped` type)

Input topic can be given by playing rosbag or by publishing a topic in real time in simulator. The topic name can be set through the launch file.

**[Execution]**

```
git clone https://github.com/Spiraline/GNSS-mapping.git
cd GNSS-mapping
catkin_make
source devel/setup.bash
roslaunch launch/gnss_mapping.launch
```

**[Parameter]**
- gnss_topic: gnss topic name
- points_topic: points topic name
- map_topic: map topic name
- voxel_leaf_size: Points is merged after filtering the points data once through the voxel grid filter. voxel leaf size is the degree of filtering. If it is 0.0, no filtering is performed at all, and the larger the value, the more filtering is performed.
- min_scan_range, max_scan_range: Use points data only within this range
- use_ndt: A flag indicating whether to use the NDT algorithm. I do not recommend using it.
- use_gnss_ori: A flag indicating whether to use orientation in GNSS topic. I recommend to use when orientation is accurate.
- min_add_scan_shift: After this value is posed from the previously merged points, the next points are merged into the map.
- output_path: output pcd file path
- tf_XX: If the position or direction between GNSS and LiDAR is different, it must be entered in tf.

**[How to save points_map into pcd file]**

```
rosparam set save_map $(voxel leaf size)
```

Before saving the map, it filters data again with the voxel grid filter.

## Example

We provide rosbag example for CubeTown map in [SVL Simulator](https://www.svlsimulator.com/).
We prepared a rosbag in which the LiDAR's position was misaligned just like the actual driving situation. Offset is as follows.
- x: 5 (m)
- y: -3 (m)
- yaw: 45 (deg)

<div style="text-align:center;">
    <img src="https://user-images.githubusercontent.com/44594966/171791867-e6d18f73-f582-445a-a43b-b03136bebe62.png" alt="example" width="500"/>
</div>

1. Configure parameters in `launch/gnss_mapping.launch`
    ```
    <!-- TF from gnss to localizer (LiDAR) -->
        <arg name="tf_x" default="5.0" />
        <arg name="tf_y" default="-3.0" />
        <arg name="tf_z" default="0.0" />
        <arg name="tf_roll" default="0.0" />
        <arg name="tf_pitch" default="0.0" />
        <arg name="tf_yaw" default="-0.785" />
    ```
    You can configure other parameters

2. Execute the launch file

    ```
    cd gnss_mapping
    roslaunch launch/gnss_mapping.launch
    ```

    You can execute `roslaunch gnss_mapping gnss_mapping.launch`, but in this case you should build (`catkin_make`) to update pararmeters.

3. Play the rosbag. You don't need to launch roscore since roslaunch already executed it.
    ```
    cd example
    rosbag play cubetown.bag
    ```
  
    This rosbag has below two topics.
    - points_raw: LiDAR points data
    - gnss_pose: GNSS data in x, y, z, quaternion format

    You can verify `gnss_pose` by `rostopic echo`

    <div style="text-align:center;">
        <img src="https://user-images.githubusercontent.com/44594966/171792311-c533f1ac-fcb7-4fd7-b009-23006fd484b2.png" alt="example" width="300"/>
    </div>

4. (Optional) You can visualize progress in Rviz when you change fixed frame into `map`.

    <div style="text-align:center;">
        <img src="https://user-images.githubusercontent.com/44594966/171795067-97b2a731-3349-49b5-86db-f4521cf3755b.png" alt="example" width="300"/>
        <img src="https://velog.velcdn.com/images/spiraline/post/bfa84386-6bcf-4810-81e7-3d2c8acac2c1/image.png" alt="example" width="400"/>
    </div>


5. Save the map

    ```
    rosparam set save_map 0.1
    ```
  
    Map is successfully saved when you see below message.

    <div style="text-align:center;">
        <img src="https://user-images.githubusercontent.com/44594966/171793425-08b1aa04-0954-4774-b8a8-eae583406adc.png" alt="example" width="300"/>
    </div>