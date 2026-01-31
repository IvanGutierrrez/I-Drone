#!/usr/bin/env python3
"""Add camera sensor to iris SDF file for Gazebo Classic."""
import sys
import xml.etree.ElementTree as ET

def add_camera_sensor(input_file, output_file, drone_id):
    """Add camera sensor to iris model."""
    
    # Parse the SDF file
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    # Find the model element
    model = root.find('.//model[@name="iris"]')
    if model is None:
        print("Error: Could not find iris model in SDF")
        sys.exit(1)
    
    # Create camera link
    camera_link = ET.SubElement(model, 'link', name='camera_link')
    
    # Pose: 10cm forward, 5cm down, pointing 90° down (pitch = π/2 = 1.5708 rad)
    pose = ET.SubElement(camera_link, 'pose')
    pose.text = '0.1 0 -0.05 0 1.5708 0'
    
    # Inertial
    inertial = ET.SubElement(camera_link, 'inertial')
    ET.SubElement(inertial, 'mass').text = '0.01'
    inertia = ET.SubElement(inertial, 'inertia')
    ET.SubElement(inertia, 'ixx').text = '0.0001'
    ET.SubElement(inertia, 'ixy').text = '0'
    ET.SubElement(inertia, 'ixz').text = '0'
    ET.SubElement(inertia, 'iyy').text = '0.0001'
    ET.SubElement(inertia, 'iyz').text = '0'
    ET.SubElement(inertia, 'izz').text = '0.0001'
    
    # Camera sensor
    sensor = ET.SubElement(camera_link, 'sensor', 
                          name=f'camera_{drone_id}',
                          type='camera')
    
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = '1.3963'  # 80 degrees (más zoom)
    
    image = ET.SubElement(camera, 'image')
    ET.SubElement(image, 'width').text = '1920'
    ET.SubElement(image, 'height').text = '1080'
    ET.SubElement(image, 'format').text = 'R8G8B8'
    
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '0.1'
    ET.SubElement(clip, 'far').text = '100'
    
    # Enable auto-save of images (Gazebo Classic feature)
    save = ET.SubElement(camera, 'save', enabled='true')
    ET.SubElement(save, 'path').text = f'/opt/I-Drone/data/drone_{drone_id}/photos'
    
    ET.SubElement(sensor, 'always_on').text = '1'
    ET.SubElement(sensor, 'update_rate').text = '1'  # 1 Hz to save CPU
    ET.SubElement(sensor, 'visualize').text = 'true'
    
    # Create fixed joint to attach camera to base_link
    joint = ET.SubElement(model, 'joint',
                         name='camera_joint',
                         type='fixed')
    ET.SubElement(joint, 'parent').text = 'base_link'
    ET.SubElement(joint, 'child').text = 'camera_link'
    
    # Format the XML with proper indentation
    ET.indent(tree, space='  ')
    
    # Write the modified SDF
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"Camera added to drone {drone_id}. Photos will save to: /opt/I-Drone/data/drone_{drone_id}/photos")

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: add_camera_to_sdf.py <input.sdf> <output.sdf> <drone_id>")
        sys.exit(1)
    
    add_camera_sensor(sys.argv[1], sys.argv[2], sys.argv[3])
