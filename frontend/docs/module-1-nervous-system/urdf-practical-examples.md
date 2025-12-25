---
sidebar_position: 6
---

# Practical Examples and Diagrams for URDF Modeling

## Overview

This section provides practical examples and hands-on demonstrations of Unified Robot Description Format (URDF) modeling for robotic systems. These examples illustrate how to create robot models, define kinematic chains, integrate sensors, and prepare robots for simulation and control.

## Basic URDF Structure

### Simple Mobile Robot Example

Let's start with a basic mobile robot model:

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black"/>
      <origin rpy="1.57075 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.57075 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black"/>
      <origin rpy="1.57075 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.57075 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Castor Wheel -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.19 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.19 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.18 0 -0.05" rpy="0 0 0"/>
  </joint>
</robot>
```

### URDF with Gazebo Integration

```xml
<?xml version="1.0"?>
<robot name="gazebo_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include Gazebo materials -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.15"/>
    </inertial>
  </link>

  <!-- Add Gazebo plugins -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.34</wheel_separation>
      <wheel_diameter>0.15</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
</robot>
```

## Advanced URDF Examples

### Manipulator Arm with Multiple Joints

```xml
<?xml version="1.0"?>
<robot name="manipulator_arm">
  <!-- Base -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Shoulder -->
  <link name="shoulder_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.3"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Upper Arm -->
  <link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0.2 0.8 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Forearm -->
  <link name="forearm_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="yellow">
        <color rgba="0.8 0.8 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Wrist -->
  <link name="wrist_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.06"/>
      </geometry>
      <material name="purple">
        <color rgba="0.6 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Gripper Base -->
  <link name="gripper_base">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.02"/>
      </geometry>
      <material name="orange">
        <color rgba="1.0 0.6 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint Definitions -->
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <joint name="shoulder_lift_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <joint name="wrist_pitch_joint" type="revolute">
    <parent link="forearm_link"/>
    <child link="wrist_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
  </joint>

  <joint name="wrist_roll_joint" type="revolute">
    <parent link="wrist_link"/>
    <child link="gripper_base"/>
    <origin xyz="0 0 0.025" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="30" velocity="1.0"/>
  </joint>
</robot>
```

### URDF with Sensors

```xml
<?xml version="1.0"?>
<robot name="sensor_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.4 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Camera Mount -->
  <link name="camera_mount">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.05"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Camera Link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.03 0.02"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
    </inertial>
  </link>

  <!-- LIDAR Mount -->
  <link name="lidar_mount">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.05"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- LIDAR Link -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0002"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="camera_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_mount"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>

  <joint name="camera_joint" type="fixed">
    <parent link="camera_mount"/>
    <child link="camera_link"/>
    <origin xyz="0 0 0.025" rpy="0 0.5 0"/>
  </joint>

  <joint name="lidar_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_mount"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>

  <joint name="lidar_joint" type="fixed">
    <parent link="lidar_mount"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo Sensor Plugins -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>600</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>camera1</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>lidar_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## URDF Best Practices and Patterns

### Using Xacro for Complex Models

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />
  <xacro:property name="base_length" value="0.5" />
  <xacro:property name="base_width" value="0.3" />
  <xacro:property name="base_height" value="0.1" />

  <!-- Macro for wheels -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.5"/>
    </inertial>
  </link>

  <!-- Use the wheel macro -->
  <xacro:wheel prefix="front_left" parent="base_link" xyz="0.15 0.15 0" rpy="0 0 0"/>
  <xacro:wheel prefix="front_right" parent="base_link" xyz="0.15 -0.15 0" rpy="0 0 0"/>
  <xacro:wheel prefix="rear_left" parent="base_link" xyz="-0.15 0.15 0" rpy="0 0 0"/>
  <xacro:wheel prefix="rear_right" parent="base_link" xyz="-0.15 -0.15 0" rpy="0 0 0"/>

</robot>
```

## URDF Validation and Testing

### Python Script for URDF Validation

```python
#!/usr/bin/env python3
"""
URDF Validation and Testing Script
"""

import xml.etree.ElementTree as ET
from collections import defaultdict
import math

class URDFValidator:
    def __init__(self, urdf_file):
        self.urdf_file = urdf_file
        self.tree = ET.parse(urdf_file)
        self.root = self.tree.getroot()

    def validate_structure(self):
        """Validate basic URDF structure"""
        errors = []

        # Check if robot element exists
        if self.root.tag != 'robot':
            errors.append("Root element must be 'robot'")

        # Check for robot name
        if 'name' not in self.root.attrib:
            errors.append("Robot element must have 'name' attribute")

        return len(errors) == 0, errors

    def validate_links(self):
        """Validate all links have required elements"""
        errors = []

        for link in self.root.findall('link'):
            link_name = link.get('name')

            # Check if link has name
            if not link_name:
                errors.append(f"Link without name found")
                continue

            # Check for visual, collision, or inertial elements
            has_visual = len(link.findall('visual')) > 0
            has_collision = len(link.findall('collision')) > 0
            has_inertial = len(link.findall('inertial')) > 0

            if not (has_visual or has_collision or has_inertial):
                errors.append(f"Link '{link_name}' has no visual, collision, or inertial elements")

        return len(errors) == 0, errors

    def validate_joints(self):
        """Validate all joints have required elements"""
        errors = []
        valid_joint_types = ['revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar']

        for joint in self.root.findall('joint'):
            joint_name = joint.get('name')
            joint_type = joint.get('type')

            # Check if joint has name and type
            if not joint_name:
                errors.append(f"Joint without name found")
            if not joint_type:
                errors.append(f"Joint '{joint_name}' without type found")
            elif joint_type not in valid_joint_types:
                errors.append(f"Joint '{joint_name}' has invalid type: {joint_type}")

            # Check for parent and child
            parent = joint.find('parent')
            child = joint.find('child')

            if parent is None:
                errors.append(f"Joint '{joint_name}' missing parent element")
            if child is None:
                errors.append(f"Joint '{joint_name}' missing child element")

        return len(errors) == 0, errors

    def validate_kinematic_chain(self):
        """Validate that the robot forms a valid kinematic chain"""
        errors = []

        # Build parent-child relationships
        joints = {}
        for joint in self.root.findall('joint'):
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            joint_type = joint.get('type')
            joints[child] = (parent, joint_type)

        # Find base link (link that is never a child)
        all_links = {link.get('name') for link in self.root.findall('link')}
        child_links = {child for child in joints.keys()}
        base_links = all_links - child_links

        if len(base_links) == 0:
            errors.append("No base link found - all links are children")
        elif len(base_links) > 1:
            errors.append(f"Multiple base links found: {base_links}")

        return len(errors) == 0, errors

    def validate_inertial_properties(self):
        """Validate inertial properties"""
        errors = []

        for link in self.root.findall('link'):
            inertial = link.find('inertial')
            if inertial is not None:
                mass = inertial.find('mass')
                if mass is None or float(mass.get('value', 0)) <= 0:
                    link_name = link.get('name')
                    errors.append(f"Link '{link_name}' has invalid mass (must be positive)")

                inertia = inertial.find('inertia')
                if inertia is not None:
                    ixx = float(inertia.get('ixx', 0))
                    iyy = float(inertia.get('iyy', 0))
                    izz = float(inertia.get('izz', 0))

                    # Check that diagonal elements are positive
                    if ixx <= 0 or iyy <= 0 or izz <= 0:
                        link_name = link.get('name')
                        errors.append(f"Link '{link_name}' has non-positive inertia diagonal elements")

        return len(errors) == 0, errors

    def run_all_validations(self):
        """Run all validations and return comprehensive report"""
        results = {}

        results['structure'] = self.validate_structure()
        results['links'] = self.validate_links()
        results['joints'] = self.validate_joints()
        results['kinematic_chain'] = self.validate_kinematic_chain()
        results['inertial'] = self.validate_inertial_properties()

        # Overall result
        all_passed = all(result[0] for result in results.values())

        return {
            'overall_passed': all_passed,
            'individual_results': results,
            'total_errors': sum(len(result[1]) for result in results.values()),
            'all_errors': [error for result in results.values() for error in result[1]]
        }

def test_urdf_model(urdf_path):
    """Test a URDF model with comprehensive validation"""
    validator = URDFValidator(urdf_path)
    results = validator.run_all_validations()

    print(f"URDF Validation Results for: {urdf_path}")
    print(f"Overall Result: {'PASS' if results['overall_passed'] else 'FAIL'}")
    print(f"Total Errors: {results['total_errors']}")

    if results['all_errors']:
        print("\nErrors found:")
        for error in results['all_errors']:
            print(f"  - {error}")

    return results['overall_passed']

# Example usage
if __name__ == "__main__":
    # This would be used to test URDF files
    pass
```

## Visualization and Debugging

### RViz Configuration for URDF

```yaml
# rviz_config.rviz
Panels:
  - Class: rviz_common/Displays
    Help Height: 85
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
        - /TF1
      Splitter Ratio: 0.5
    Tree Height: 643
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Name: RobotModel
      Robot Description: robot_description
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.5
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        {}
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.5
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.5
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Width: 1200
```

## Practical Exercises

### Exercise 1: Create a Simple Differential Drive Robot

Create a URDF file for a simple differential drive robot with:
- A rectangular base
- Two driven wheels
- One castor wheel
- Proper inertial properties

### Exercise 2: Add Sensors to Your Robot

Extend your robot model by adding:
- A camera on a mast
- A LIDAR sensor
- Proper Gazebo plugins for simulation

### Exercise 3: Validate Your URDF

Use the validation script to check your URDF model and fix any errors found.

These practical examples demonstrate the key concepts of URDF modeling for robotic systems, from basic structure to advanced features with sensors and simulation integration.