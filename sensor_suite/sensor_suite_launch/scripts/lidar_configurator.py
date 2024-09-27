#!/usr/bin/env python

import rospy
import json
import requests
from sensor_msgs.msg import PointCloud2
from ouster_ros.srv import GetConfig

# "multipurpose_io_mode": "OUTPUT_FROM_INTERNAL_OSC
class LidarConfigurator:
    def __init__(self):
        self.message_count = 0
        self.target_message_count = 100
        self.new_messages_received = False

        # New configuration values for specific keys
        self.new_config_values = {
            "lidar_mode": "1024x10",
            "sync_pulse_out_frequency": 10,
            "multipurpose_io_mode": "OUTPUT_FROM_ENCODER_ANGLE"
        }

        self.lidar_ip = "192.168.10.210"
        self.api_endpoint = f"http://{self.lidar_ip}/api/v1/sensor/config"

        # Initialize ROS node
        rospy.init_node("ouster_lidar_config")

        # Subscribe to the points topic
        self.subscriber = rospy.Subscriber('/sensor_suite/ouster/points', PointCloud2, self.points_callback)

    def points_callback(self, msg):
        self.message_count += 1
        if self.message_count >= self.target_message_count:
            self.new_messages_received = True
            self.subscriber.unregister()  # Unsubscribe from the topic

    def configure_lidar(self):
        try:
            # Wait for the get_config service to become available
            rospy.wait_for_service('/sensor_suite/ouster/get_config')

            # Create service proxy
            get_config = rospy.ServiceProxy('/sensor_suite/ouster/get_config', GetConfig)

            # Get current configuration
            current_config_response = get_config()
            current_config = json.loads(current_config_response.config)

            rospy.loginfo("Current lidar configuration:")
            rospy.loginfo(json.dumps(current_config, indent=2))

            # Update only the specified keys
            for key, value in self.new_config_values.items():
                if key in current_config:
                    current_config[key] = value

            # Apply the updated configuration using HTTP POST request
            response = requests.post(self.api_endpoint, json=current_config)

            # Check the response status code
            if response.status_code == 200 or response.status_code == 204:
                rospy.loginfo("Lidar configuration applied successfully.")

                # Wait for new points to be published on the /sensor_suite/ouster/points topic
                rospy.loginfo(f"Waiting for {self.target_message_count} new messages on /sensor_suite/ouster/points...")
                while not self.new_messages_received and not rospy.is_shutdown():
                    rospy.sleep(0.2)

                rospy.loginfo("Received 20 new lidar messages.")
                rospy.sleep(30.)

                # Retrieve the updated configuration to verify changes
                updated_config_response = get_config()
                updated_config_response = get_config()
                updated_config_response = get_config()
                updated_config = json.loads(updated_config_response.config)

                rospy.loginfo("Updated lidar configuration:")
                rospy.loginfo(json.dumps(updated_config, indent=2))

                # Verify that the updated configuration matches the requested changes
                for key, value in self.new_config_values.items():
                    if key in updated_config and updated_config[key] == value:
                        rospy.loginfo(f"The key '{key}' was successfully updated to '{value}' in the final configuration.")
                    else:
                        rospy.logwarn(f"The key '{key}' was not updated correctly in the final configuration. Requested: '{value}', Final: '{updated_config.get(key, 'N/A')}'")
            else:
                rospy.logerr(f"Lidar configuration failed. Status code: {response.status_code}")
                rospy.logerr(f"Response content: {response.text}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"HTTP request failed: {e}")

if __name__ == "__main__":
    configurator = LidarConfigurator()
    configurator.configure_lidar()
    rospy.spin()
