# Chapter 3: Simulating Robot Sensors

## Introduction to Sensor Simulation

Sensors are a robot's eyes, ears, and touch. They provide crucial information about the robot's internal state and its surrounding environment, enabling perception, navigation, and interaction. However, real-world sensors can be expensive, fragile, and require careful calibration. This is where **sensor simulation** becomes invaluable.

Simulated sensors allow us to:

*   **Develop and test algorithms**: Write and debug perception algorithms (e.g., for object detection, SLAM) without needing physical hardware.
*   **Generate synthetic data**: Create large, diverse datasets for training machine learning models, overcoming the limitations of real-world data collection.
*   **Experiment with configurations**: Rapidly test different sensor placements, types, and parameters without physical modifications.
*   **Understand sensor limitations**: Analyze how noise, occlusions, and environmental factors affect sensor readings.

This chapter will guide you through configuring common robot sensors in both Gazebo and Unity, focusing on LiDAR, Depth Cameras, and IMUs.

## Adding a LiDAR Sensor in Gazebo

**LiDAR (Light Detection and Ranging)** sensors measure distances to objects by emitting pulsed laser light and measuring the time it takes for the reflected light to return. They are crucial for 3D mapping, obstacle avoidance, and navigation.

In Gazebo, you can add a simulated LiDAR using a plugin. This usually involves modifying your robot's SDF or URDF file.

```xml
<link name="lidar_link">
  <visual>
    <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
  </visual>
  <collision>
    <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
  </collision>
</link>

<joint name="lidar_joint" type="fixed">
  <parent>base_link</parent> <!-- Attach to your robot's base link -->
  <child>lidar_link</child>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>
          <max_angle>1.570796</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <argument>~/out:=scan</argument>
        <namespace>/my_robot</namespace>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```
*Self-check*: After adding this to your robot model and launching the robot in Gazebo, you should see a new ROS 2 topic `/my_robot/scan` publishing `sensor_msgs/LaserScan` messages.

## Adding a Depth Camera Plugin in Gazebo

**Depth Cameras** provide both RGB (color) images and a depth map (distance to objects). This combination is invaluable for 3D reconstruction, object recognition, and grasping tasks.

Gazebo supports depth cameras through plugins. Here's a typical configuration snippet:

```xml
<link name="depth_camera_link">
  <visual>
    <geometry><box><size>0.03 0.03 0.03</size></box></geometry>
  </visual>
  <collision>
    <geometry><box><size>0.03 0.03 0.03</size></box></geometry>
  </collision>
</link>

<joint name="depth_camera_joint" type="fixed">
  <parent>base_link</parent>
  <child>depth_camera_link</child>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
</joint>

<gazebo reference="depth_camera_link">
  <sensor name="depth_camera_sensor" type="depth">
    <always_on>1</always_on>
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>3.0</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_depth_camera.so">
      <ros>
        <argument>image_raw:=depth/image_raw</argument>
        <argument>camera_info:=depth/camera_info</argument>
        <argument>depth/image_raw:=depth/depth_image_raw</argument>
        <namespace>/my_robot/camera</namespace>
      </ros>
      <frame_name>depth_camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```
*Self-check*: When launched, you should see ROS 2 topics like `/my_robot/camera/depth/image_raw` and `/my_robot/camera/depth/depth_image_raw` publishing image messages.

## Adding an IMU Sensor Plugin in Gazebo

An **IMU (Inertial Measurement Unit)** measures a robot's specific force (acceleration) and angular velocity. This data is critical for estimating the robot's orientation, velocity, and position, especially in navigation and control tasks.

Gazebo IMU plugins simulate accelerometers and gyroscopes:

```xml
<link name="imu_link">
  <visual>
    <geometry><box><size>0.01 0.01 0.01</size></box></geometry>
  </visual>
  <collision>
    <geometry><box><size>0.01 0.01 0.01</size></box></geometry>
  </collision>
</link>

<joint name="imu_joint" type="fixed">
  <parent>base_link</parent>
  <child>imu_link</child>
  <origin xyz="0 0 0.16" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>100</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/my_robot/imu</namespace>
        <argument>~/out:=imu_data</argument>
      </ros>
      <frame_name>imu_link</frame_name>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```
*Self-check*: The topic `/my_robot/imu/imu_data` should publish `sensor_msgs/Imu` messages with `linear_acceleration` and `angular_velocity` data.

## Adding a Camera Sensor with Unity Perception Package

The **Unity Perception package** is designed for generating synthetic datasets for computer vision tasks. It allows you to add various types of camera sensors (RGB, Depth, Semantic Segmentation, etc.) to your Unity scene and configure them to output data in a structured format suitable for machine learning.

1.  **Install Perception Package**:
    *   In your Unity project, go to `Window > Package Manager`.
    *   Click the `+` icon and choose "Add package from git URL...".
    *   Enter `com.unity.perception` and click "Add".

2.  **Add a Camera to your Scene**:
    *   Right-click in the Hierarchy window.
    *   `3D Object > Camera`. Position the camera to simulate a robot's view.
    *   Select the camera, then in the Inspector, click "Add Component" and search for "Perception Camera". Add it.

3.  **Configure Perception Camera**:
    *   **Capture Interval**: How often the camera captures data.
    *   **Output Directory**: Where the captured data will be saved.
    *   **Labelers**: Add labelers (e.g., `RGB Camera Labeler`, `Depth Camera Labeler`) to specify what type of data to generate.
    *   **Camera Parameters**: Adjust FOV, clip planes, etc.

When you run your Unity scene, the Perception Camera will save the specified sensor data (e.g., RGB images) to your output directory, ready for use in AI training.

## Visualizing Gazebo Sensor Data with RViz2

**RViz2** is a 3D visualization tool for ROS 2. It allows you to display various types of robot data, including sensor readings from Gazebo, in a graphical environment.

1.  **Launch RViz2**:
    ```bash
    rviz2
    ```

2.  **Add Displays**:
    *   In the RViz2 window, click the "Add" button in the "Displays" panel (bottom left).
    *   **For LiDAR**: Select `LaserScan` and set the topic to `/my_robot/scan`.
    *   **For Depth Camera**: Select `Image` and set the topic to `/my_robot/camera/depth/image_raw` or `/my_robot/camera/depth/depth_image_raw`.
    *   **For IMU**: Select `IMU` and set the topic to `/my_robot/imu/imu_data`.
    *   **Set Fixed Frame**: In the "Global Options" section, set the "Fixed Frame" to `map` or your robot's base frame (e.g., `base_link`).

By visualizing the sensor data in RViz2, you can verify that your Gazebo sensors are correctly configured and are publishing the expected information, providing a crucial feedback loop for your simulation setup.

This concludes Module 2 on building digital twins and simulating sensors.
