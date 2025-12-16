# Chapter 3: Building the Robot Body in URDF

## What URDF is and Why Humanoid Robots Need It

The **Unified Robot Description Format (URDF)** is an XML file format used in ROS to describe all elements of a robot. It's a fundamental tool for any robot developer, especially for complex systems like humanoids. A URDF file completely specifies the robot's:

*   **Kinematic and Dynamic Properties**: Describes the robot's structure, including its links (rigid bodies) and joints (connections between links). It also defines inertial properties (mass, inertia matrix) for each link.
*   **Visual Properties**: Defines the visual appearance of each link, including its 3D mesh model, color, and texture. This is used for rendering the robot in simulators (like Gazebo) or visualization tools (like RViz).
*   **Collision Properties**: Defines simplified geometric shapes used for collision detection in simulations. These are often simpler than visual meshes to speed up collision calculations.

For humanoid robots, URDF is indispensable because:

*   **Complexity**: Humanoids have many links and joints (legs, arms, torso, head), making a detailed, systematic description essential.
*   **Simulation**: URDF files are directly used by physics simulators (e.g., Gazebo) to model the robot's physical behavior.
*   **Visualization**: Tools like RViz use URDF to display the robot's structure and its current state (e.g., joint angles).
*   **Kinematics/Dynamics Libraries**: Software packages for inverse kinematics or dynamics calculations parse URDF to understand the robot's structure.

## Links, Joints, and Kinematic Chains

A URDF model is built up from two primary elements: `<link>` and `<joint>`.

### The `<link>` Element: Rigid Bodies

A `<link>` element describes a rigid body part of the robot. This could be a robot's torso, an upper arm, a thigh, or a foot. Each link has:

*   **Inertial Properties**: Defined by the `<inertial>` tag, including `mass` and the `inertia` matrix. These are crucial for accurate physics simulation.
*   **Visual Properties**: Defined by the `<visual>` tag, specifying the geometry (mesh, box, cylinder, sphere) and material (color, texture) of the link.
*   **Collision Properties**: Defined by the `<collision>` tag, specifying the geometry used for collision detection. This is often a simplified shape of the visual.

### The `<joint>` Element: Connections Between Links

A `<joint>` element describes the kinematic and dynamic properties of the connection between two links. Joints define how links can move relative to each other. Key attributes include:

*   **`name`**: A unique identifier for the joint.
*   **`type`**: Specifies the joint's movement capabilities (e.g., `revolute` for a rotating joint like a hinge, `prismatic` for a sliding joint, `fixed` for a rigid connection).
*   **`parent`**: The name of the link closer to the robot's base or origin.
*   **`child`**: The name of the link further away from the robot's base.
*   **`origin`**: Defines the joint's position and orientation relative to the parent link.
*   **`axis`**: For revolute and prismatic joints, specifies the axis of rotation or translation.
*   **`limit`**: Defines the joint's movement range (lower/upper limits), velocity, and effort limits.

### Kinematic Chains

A series of links connected by joints forms a **kinematic chain**. For humanoids, you'll have multiple chains: one for each leg, one for each arm, one for the torso/head, all branching from a central base link (e.g., the torso or hip).

## Creating a Basic Humanoid URDF Model

Let's create a very simple URDF model for a two-legged robot. This will demonstrate the basic structure of links and joints. This model will be saved as `Humanoid_Robotics/simulations/module-1/simple_humanoid.urdf`.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link: Torso -->
  <link name="torso">
    <visual>
      <geometry><box size="0.2 0.1 0.3"/></geometry>
      <material name="blue"><color rgba="0 0 1 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.2 0.1 0.3"/></geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <link name="left_thigh">
    <visual>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
      <material name="green"><color rgba="0 1 0 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="torso_to_left_thigh" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="0 0.075 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="left_thigh_to_left_shin" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="0" effort="100" velocity="10"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry><box size="0.07 0.05 0.03"/></geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry><box size="0.07 0.05 0.03"/></geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>
  <joint name="left_shin_to_left_foot" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.785" upper="0.785" effort="100" velocity="10"/>
  </joint>

  <!-- Right Leg (similar structure, just mirrored in Y) -->
  <link name="right_thigh">
    <visual>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="torso_to_right_thigh" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="0 -0.075 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <link name="right_shin">
    <visual>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="right_thigh_to_right_shin" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="0" effort="100" velocity="10"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry><box size="0.07 0.05 0.03"/></geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry><box size="0.07 0.05 0.03"/></geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>
  <joint name="right_shin_to_right_foot" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.785" upper="0.785" effort="100" velocity="10"/>
  </joint>

  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="green">
    <color rgba="0 1 0 1"/>
  </material>

</robot>
