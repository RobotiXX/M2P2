#!/usr/bin/env python3

import json
import yaml
import rosbag

# Example script to create a connection header template
                             
# Get a connection_header for a tf_static msg
for topic, msg, stamp, connection_header in rosbag.Bag("rosbags/campus_good_light_7.bag_2024-08-25_20-04-45.bag").read_messages(return_connection_header=True):
    if topic == '/tf_static':
        tf_static_ch = connection_header
        break


# Get a connection_header for a camera_info msg
for topic, msg, stamp, connection_header in rosbag.Bag("/media/mocap/Data/Classical_nav_2024-09-11-21-33-27.bag").read_messages(return_connection_header=True):
    if topic == '/sensor_suite/left_camera_optical/camera_info':
        cam_info_ch = connection_header
        break

# left_camera_info_connection_header['topic'] = b'/sensor_suite/lwir/lwir/camera_info'
# Serialize the connection_header to yaml
with open('connection_headers/tf_static_ch.yaml', 'w') as f:
    yaml.dump(tf_static_ch, f)

with open('connection_headers/cam_info_ch.yaml', 'w') as f:
    yaml.dump(cam_info_ch, f)

# Deserialize the connection_header from yaml
with open('connection_headers/tf_static_ch.yaml', 'r') as f:
    loaded_connection_header = yaml.safe_load(f)

print(loaded_connection_header['latching'])
print(loaded_connection_header['topic'])
print(loaded_connection_header['type'])
print(loaded_connection_header['md5sum'])


with open('connection_headers/cam_info_ch.yaml', 'r') as f:
    loaded_connection_header = yaml.safe_load(f)

print(loaded_connection_header['latching'])
print(loaded_connection_header['topic'])
print(loaded_connection_header['type'])
print(loaded_connection_header['md5sum'])
