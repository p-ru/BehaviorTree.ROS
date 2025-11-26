#!/usr/bin/env python3
import rospy
import json
import os
import yaml  # Requires PyYAML (standard in ROS)
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class MagicOdomTrigger:
    def __init__(self):
        rospy.init_node('mavic_odom_trigger', anonymous=True)

        # --- Configuration ---
        self.file_path = '/workspaces/BT_projects/src/behaviortree_ros/config/new_waypoints.yaml'
        
        # Variable to store the latest odometry message
        self.current_odom = None

        # Subscriber 1: Listen to the TagSLAM odometry
        rospy.Subscriber("/tagslam/odom/body_rig", Odometry, self.odom_callback)

        # Subscriber 2: Listen to the Mavic state (Trigger)
        rospy.Subscriber("/Mavic_3M/state/other", String, self.mavic_callback)

        # Define the target command we are waiting for
        self.target_command = {
            "operation": "listen",
            "channel": "REMOTE_CONTROLLER",
            "key": "CustomButton3Down",
            "param": True
        }

        rospy.loginfo(f"Node initialized. Waypoints will be saved to: {self.file_path}")
        rospy.spin()

    def odom_callback(self, msg):
        """Updates the latest known position."""
        self.current_odom = msg

    def mavic_callback(self, msg):
        """Checks incoming Mavic messages and triggers the save action."""
        try:
            received_data = json.loads(msg.data)
            if received_data == self.target_command:
                self.process_trigger()
        except json.JSONDecodeError:
            pass
        except Exception as e:
            rospy.logerr(f"Error processing message: {e}")

    def process_trigger(self):
        """Handles the trigger: prints to terminal and saves to YAML."""
        if self.current_odom is None:
            rospy.logwarn("Trigger received, but no Odometry data has been received yet!")
            return

        # 1. Get Data
        pos = self.current_odom.pose.pose.position
        
        # 2. Prepare the data list
        waypoints_list = []
        
        # Check if file exists and load current waypoints to append to them
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, 'r') as f:
                    loaded_data = yaml.safe_load(f)
                    if isinstance(loaded_data, list):
                        waypoints_list = loaded_data
            except yaml.YAMLError as e:
                rospy.logwarn(f"Error reading existing YAML, starting fresh: {e}")

        # 3. Generate ID and New Entry
        # We auto-generate an ID based on the current list length (e.g., waypoint_0, waypoint_1)
        # You can change this logic if you need specific naming (like "start_point")
        new_id = f"waypoint_{len(waypoints_list)}"
        
        new_entry = {
            'x': round(float(pos.x), 4),
            'y': round(float(pos.y), 4),
            'z': round(float(pos.z), 4),
            'id': new_id
        }

        waypoints_list.append(new_entry)

        # 4. Save to File
        try:
            # Ensure the directory exists
            os.makedirs(os.path.dirname(self.file_path), exist_ok=True)
            
            with open(self.file_path, 'w') as f:
                # default_flow_style=False creates the block format (bullet points)
                # sort_keys=False keeps the order (x, y, z, id) as defined in the dict
                yaml.dump(waypoints_list, f, default_flow_style=False, sort_keys=False)
                
            rospy.loginfo(f"Saved {new_id} to file.")
            print(f"Captured -> x: {new_entry['x']}, y: {new_entry['y']}, z: {new_entry['z']}, id: {new_entry['id']}")
            
        except Exception as e:
            rospy.logerr(f"Failed to write to file: {e}")

if __name__ == '__main__':
    try:
        MagicOdomTrigger()
    except rospy.ROSInterruptException:
        pass
