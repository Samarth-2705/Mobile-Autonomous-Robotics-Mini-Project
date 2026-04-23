#!/usr/bin/env python3
"""
Run this script ONCE to generate:
  1. The ArUco marker PNG image
  2. Gazebo material script (.material)
  3. Updated platform URDF using the texture

Usage:
    python3 generate_marker.py

Then rebuild:
    cd ~/drone_ws && colcon build --symlink-install
"""

import cv2
import os
import numpy as np

# ------------------------------------------------------------------ #
# 1. Generate the ArUco marker image
# ------------------------------------------------------------------ #
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Try new API first, fall back to old
try:
    marker_img = cv2.aruco.generateImageMarker(dictionary, 0, 512)
except AttributeError:
    marker_img = np.zeros((512, 512), dtype=np.uint8)
    cv2.aruco.drawMarker(dictionary, 0, 512, marker_img, 1)

# Add white border (improves detection)
bordered = cv2.copyMakeBorder(
    marker_img, 40, 40, 40, 40,
    cv2.BORDER_CONSTANT, value=255)

out_dir = os.path.expanduser(
    '~/drone_ws/src/drone_landing/models/aruco_platform')
os.makedirs(out_dir, exist_ok=True)

img_path = os.path.join(out_dir, 'aruco_0.png')
cv2.imwrite(img_path, bordered)
print(f'✓ Marker image saved: {img_path}')

# ------------------------------------------------------------------ #
# 2. Write model.config
# ------------------------------------------------------------------ #
model_config = """<?xml version="1.0"?>
<model>
  <name>aruco_platform</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>Moving landing platform with ArUco marker ID 0</description>
</model>
"""
with open(os.path.join(out_dir, 'model.config'), 'w') as f:
    f.write(model_config)
print('✓ model.config written')

# ------------------------------------------------------------------ #
# 3. Write model.sdf — platform with ArUco texture on top face
# ------------------------------------------------------------------ #
model_sdf = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="aruco_platform">
    <static>false</static>

    <link name="platform_base">
      <inertial>
        <mass>50.0</mass>
        <inertia>
          <ixx>4.17</ixx><iyy>4.17</iyy><izz>8.33</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <!-- Green platform body -->
      <visual name="body">
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>1.0 1.0 0.05</size></box></geometry>
        <material>
          <ambient>0.0 0.7 0.1 1</ambient>
          <diffuse>0.0 0.8 0.2 1</diffuse>
        </material>
      </visual>

      <!-- ArUco marker face on top -->
      <visual name="aruco_marker">
        <pose>0 0 0.026 0 0 0</pose>
        <geometry><box><size>0.6 0.6 0.001</size></box></geometry>
        <material>
          <script>
            <uri>model://aruco_platform/marker_material.material</uri>
            <name>ArUcoMarker</name>
          </script>
        </material>
      </visual>

      <collision name="collision">
        <geometry><box><size>1.0 1.0 0.05</size></box></geometry>
        <surface>
          <contact><ode/></contact>
          <friction><ode><mu>1</mu><mu2>1</mu2></ode></friction>
        </surface>
      </collision>
    </link>

    <!-- Planar move plugin -->
    <plugin name="platform_move" filename="libgazebo_ros_planar_move.so">
      <ros>
        <namespace>platform</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <update_rate>100</update_rate>
      <publish_rate>20</publish_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>platform_base</robot_base_frame>
    </plugin>

  </model>
</sdf>
"""
with open(os.path.join(out_dir, 'model.sdf'), 'w') as f:
    f.write(model_sdf)
print('✓ model.sdf written')

# ------------------------------------------------------------------ #
# 4. Write Gazebo material script
# ------------------------------------------------------------------ #
material_script = """material ArUcoMarker
{
    technique
    {
        pass
        {
            texture_unit
            {
                texture aruco_0.png
                filtering none
            }
        }
    }
}
"""
with open(os.path.join(out_dir, 'marker_material.material'), 'w') as f:
    f.write(material_script)
print('✓ material script written')

# ------------------------------------------------------------------ #
# 5. Update GAZEBO_MODEL_PATH in ~/.bashrc
# ------------------------------------------------------------------ #
models_dir = os.path.expanduser('~/drone_ws/src/drone_landing/models')
bashrc_path = os.path.expanduser('~/.bashrc')

export_line = f'\nexport GAZEBO_MODEL_PATH={models_dir}:$GAZEBO_MODEL_PATH\n'

with open(bashrc_path, 'r') as f:
    bashrc_content = f.read()

if 'aruco_platform' not in bashrc_content and models_dir not in bashrc_content:
    with open(bashrc_path, 'a') as f:
        f.write(export_line)
    print(f'✓ Added GAZEBO_MODEL_PATH to ~/.bashrc')
    print(f'  → Run: source ~/.bashrc')
else:
    print('✓ GAZEBO_MODEL_PATH already set in ~/.bashrc')

print('\n✅ All done!')
print('Next steps:')
print('  1. source ~/.bashrc')
print('  2. cd ~/drone_ws && colcon build --symlink-install')
print('  3. source install/setup.bash')
print('  4. ros2 launch drone_landing simulation.launch.py')
