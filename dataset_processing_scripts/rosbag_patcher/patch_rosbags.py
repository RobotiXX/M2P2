#!/usr/bin/env python3

import argparse
import rosbag
import rospy
import yaml
import os
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from tqdm import tqdm

# README : This script can be used to patch /tf_static and camera_info topics into rosbags.
# Intended usecase for this script is to add intrinsic and extrinsic calibration data into rosbags.
# Intrinsic calibration matrices are retrieved from kalibr camchain yaml
# Extrinsic calibration is applied in the form of a precomputed tf_static.yaml
# Applies a connection header from a "donor" rosbag to get around a problem that causes the newly added messages to not be recognized properly

def load_tf_static_from_yaml(yaml_path):
    with open(yaml_path, 'r') as f:
        tf_dict = yaml.safe_load(f)
    
    tf_static = TFMessage()
    for transform_dict in tf_dict['transforms']:
        transform = TransformStamped()
        transform.header.frame_id = transform_dict['header']['frame_id']
        transform.child_frame_id = transform_dict['child_frame_id']
        transform.transform.translation.x = transform_dict['transform']['translation']['x']
        transform.transform.translation.y = transform_dict['transform']['translation']['y']
        transform.transform.translation.z = transform_dict['transform']['translation']['z']
        transform.transform.rotation.x = transform_dict['transform']['rotation']['x']
        transform.transform.rotation.y = transform_dict['transform']['rotation']['y']
        transform.transform.rotation.z = transform_dict['transform']['rotation']['z']
        transform.transform.rotation.w = transform_dict['transform']['rotation']['w']
        tf_static.transforms.append(transform)
    
    return tf_static

def load_camchain(camchain_path):
    with open(camchain_path, 'r') as f:
        camchain = yaml.safe_load(f)
    return camchain

def get_frame_id(camera_name):
    cam_to_frame_id = {"cam0": "right_camera_optical",
                          "cam1": "left_camera_optical",
                          "cam2": "ir_camera_optical",
                          "cam3": "event_camera_optical"}
    frame_id = cam_to_frame_id.get(camera_name, f"{camera_name}_optical")
    return frame_id.decode('utf-8') if isinstance(frame_id, bytes) else frame_id

def create_camera_info(camera_name, camera_params, timestamp):
    camera_info = CameraInfo()
    camera_info.header.stamp = timestamp
    frame_id = get_frame_id(camera_name)
    camera_info.header.frame_id = frame_id

    # Image dimensions
    camera_info.height = camera_params['resolution'][1]
    camera_info.width = camera_params['resolution'][0]

    # Distortion model and coefficients
    camera_info.distortion_model = "plumb_bob"
    camera_info.D = camera_params.get('distortion_coeffs', [])

    # Intrinsic camera matrix K
    intrinsics = camera_params.get('intrinsics', [])
    camera_model = camera_params.get('camera_model', 'pinhole')
    
    if camera_model == 'pinhole':
        if len(intrinsics) != 4:
            rospy.logwarn(f"Camera {camera_name} has invalid intrinsics. Expected 4 values for pinhole model.")
            return None
        fx, fy, px, py = intrinsics
    elif camera_model == 'omni':
        if len(intrinsics) != 5:
            rospy.logwarn(f"Camera {camera_name} has invalid intrinsics. Expected 5 values for omni model.")
            return None
        xi, fx, fy, px, py = intrinsics
    elif camera_model == 'ds':
        if len(intrinsics) != 6:
            rospy.logwarn(f"Camera {camera_name} has invalid intrinsics. Expected 6 values for ds model.")
            return None
        xi, alpha, fx, fy, px, py = intrinsics
    elif camera_model == 'eucm':
        if len(intrinsics) != 6:
            rospy.logwarn(f"Camera {camera_name} has invalid intrinsics. Expected 6 values for eucm model.")
            return None
        alpha, beta, fx, fy, px, py = intrinsics
    else:
        rospy.logwarn(f"Camera {camera_name} has unsupported camera model: {camera_model}")
        return None

    # Construct the intrinsic matrix K
    camera_info.K = [
        fx, 0.0, px,
        0.0, fy, py,
        0.0, 0.0, 1.0
    ]

    # Rectification matrix R (identity for monocular cameras)
    camera_info.R = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    ]

    # Projection matrix P
    camera_info.P = [
        fx, 0.0, px, 0.0,
        0.0, fy, py, 0.0,
        0.0, 0.0, 1.0, 0.0
    ]

    # # handle P for RGB cameras
    # if camera_name == "cam0":
    #     camera_info.P[3] = fx * 0.10  # Tx for right camera
    #     camera_info.P[7] = 0.0
    # elif camera_name == "cam1":
    #     camera_info.P[3] = fx * -0.10
    #     camera_info.P[7] = 0.0

    return camera_info

def is_camera_info_valid(camera_info):
    # Check essential fields to ensure camera_info is not empty
    if camera_info.width == 0 or camera_info.height == 0:
        return False
    if all(k == 0.0 for k in camera_info.K):
        return False
    # Allow cameras with no distortion (empty D)
    if camera_info.D and all(d == 0.0 for d in camera_info.D):
        return False
    return True

def process_bag(input_bag_path, output_bag_path, tf_static, camchain, image_to_info_mapping, tf_static_connection_header_path, cam_info_connection_header_path):
    first_tf_time = None
    tf_static_written = False

    with open(tf_static_connection_header_path, 'r') as f:
        tf_static_connection_header = yaml.safe_load(f)

    with open(cam_info_connection_header_path, 'r') as f:
        cam_info_connection_header = yaml.safe_load(f)

    with rosbag.Bag(output_bag_path, 'w') as outbag:
        with rosbag.Bag(input_bag_path) as inbag:
            for topic, msg, t, connection_header in tqdm(inbag.read_messages(return_connection_header=True), desc=f"Processing {os.path.basename(input_bag_path)}"):
                if topic == "/tf" and not tf_static_written:
                    first_tf_time = t
                    
                    # Use connection header from /tf message directly for /tf_static.
                    outbag.write("/tf_static", tf_static, first_tf_time,
                                 connection_header=tf_static_connection_header)
                    tf_static_written = True

                # Check if the current topic is one of the image topics.
                for cam, topics in image_to_info_mapping.items():
                    if topic == topics['image_topic']:
                        camera_info_topic = topics['camera_info']
                        camera_params = camchain.get(cam)
                        if camera_params:
                            camera_info = create_camera_info(cam,camera_params,t)
                            if is_camera_info_valid(camera_info):
                                # Modify connection header specifically for CameraInfo.
                                # modified_connection_header=modify_connection_header_for_camera_info(connection_header)
                                camera_info_topic_bytes = camera_info_topic.encode('utf-8')
                                cam_info_connection_header['topic'] = camera_info_topic_bytes
                                outbag.write(camera_info_topic,camera_info,t ,connection_header=cam_info_connection_header)
                        else :
                            rospy.logwarn(f"Camera parameters for {cam} not found in camchain.")
                        break 

                if topic != "/tf_static": 
                    outbag.write(topic,msg,t ,connection_header=connection_header)

            # If no /tf message was found but messages exist in bag.
            # Mostly redundant as /tf messages are expected to be present.
            if not tf_static_written and inbag.get_message_count() > 0:
                last_time=t 
                for transform in tf_static.transforms:
                    transform.header.stamp=last_time 
                outbag.write("/tf_static",tf_static,last_time)

def validate_bag(output_bag_path,camera_info_topics):
   validation_passed=True 
   with rosbag.Bag(output_bag_path)as bag: 
       topic_info=bag.get_type_and_topic_info()[1]
       for cam ,info_topic in camera_info_topics.items():
           if info_topic not in topic_info: 
               print(f"Validation Failed: {info_topic} not found in {output_bag_path}")
               validation_passed=False 
           else :
               msg_type=topic_info[info_topic].msg_type 
               if msg_type!="sensor_msgs/CameraInfo":
                   print(f"Validation Failed: {info_topic} has incorrect type {msg_type} in {output_bag_path}")
                   validation_passed=False 
   if validation_passed: 
       print(f"Validation Passed: All CameraInfo topics are present with correct types in {output_bag_path}")
   return validation_passed 

def process_folder(input_folder ,output_folder ,tf_static_yaml_path ,camchain_path ,image_to_info_mapping, tf_static_connection_header_path, cam_info_connection_header_path):
   tf_static=load_tf_static_from_yaml(tf_static_yaml_path)
   camchain=load_camchain(camchain_path)

   if not os.path.exists(output_folder): 
       os.makedirs(output_folder)

   for filename in os.listdir(input_folder): 
       if filename.endswith(".bag"):
           input_path=os.path.join(input_folder ,filename)
           output_path=os.path.join(output_folder,f"processed_{filename}")
           process_bag(input_path ,output_path ,tf_static ,camchain ,image_to_info_mapping, tf_static_connection_header_path, cam_info_connection_header_path)
           print(f"Processed {filename}")
           validate_bag(output_path,{cam :info['camera_info']for cam ,info in image_to_info_mapping.items()})

   print("All bags processed successfully.")

if __name__=="__main__":
   
    # Set up the argument parser
    parser = argparse.ArgumentParser(description='Patch rosbags with camera_info and tf_static')

    # Add arguments
    parser.add_argument('--input_folder', type=str, default='./rosbags', help='Path to the input folder')
    parser.add_argument('--output_folder', type=str, default='./processed_rosbags', help='Path to the output folder')
    parser.add_argument('--tf_static_yaml_path', type=str, default='tf_static.yaml', help='Path to the static TF YAML file')
    parser.add_argument('--camchain_path', type=str, default='calib/wfov_i-camchain_2.yaml', help='Path to the camera chain file')
    parser.add_argument('--tf_static_connection_header_path', type=str, default='connection_headers/tf_static_ch.yaml', help='Path to the TF static connection header')
    parser.add_argument('--cam_info_connection_header_path', type=str, default='connection_headers/cam_info_ch.yaml', help='Path to the camera info connection header')

    # Parse the arguments
    args = parser.parse_args()

    # Use the arguments in your script
    input_folder = args.input_folder
    output_folder = args.output_folder
    tf_static_yaml_path = args.tf_static_yaml_path
    camchain_path = args.camchain_path
    tf_static_connection_header_path = args.tf_static_connection_header_path
    cam_info_connection_header_path = args.cam_info_connection_header_path


    image_to_info_mapping={
        "cam0":{
            "image_topic":"/sensor_suite/right_camera_optical/image_color/compressed",
            "camera_info":"/sensor_suite/right_camera_optical/camera_info"
        },
        "cam1":{
            "image_topic":"/sensor_suite/left_camera_optical/image_color/compressed",
            "camera_info":"/sensor_suite/left_camera_optical/camera_info"
        },
        "cam2":{
            "image_topic":"/sensor_suite/lwir/lwir/image_raw/compressed",
            "camera_info":"/sensor_suite/lwir/lwir/camera_info"
        },
        "cam3":{
            "image_topic":"/sensor_suite/event_camera/events",
            "camera_info":"/sensor_suite/event_camera/camera_info"
        }
    }
    process_folder(input_folder ,output_folder ,tf_static_yaml_path ,camchain_path ,image_to_info_mapping, tf_static_connection_header_path, cam_info_connection_header_path)