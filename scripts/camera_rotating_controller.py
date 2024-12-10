#!/usr/bin/env python3

import roslib
import rospy
import time
import math
import sys
import numpy as np
from scipy.ndimage import filters
import imutils
import cv2

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Int32, Float64
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CompressedImage, CameraInfo

class MarkerRotationController:

    def __init__(self):
        rospy.init_node('marker_rotation_node')
        
        self.camera_center = Point()
        self.marker_position = Point()
        self.collected_marker_ids = []
        self.markers_location_map = {}  # Stores marker ID to (x,y) center position
        self.observed_marker_id = 0
        self.collection_phase = True  # True while collecting marker IDs
        self.alignment_achieved = False
        self.active_marker = 0
        self.current_rotation_angle = 0.0
        
        # Publishers
        self.processed_image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.camera_holder_pub = rospy.Publisher('/my_robot4/camera_holder_position_controller/command', Float64, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1)
        rospy.Subscriber("/robot/camera1/camera_info", CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber("/marker/id_number", Int32, self.marker_id_callback, queue_size=1)
        rospy.Subscriber("/marker/center_loc", Point, self.marker_center_callback, queue_size=1)

    def camera_info_callback(self, msg):
        # Calculate the camera's center point
        self.camera_center.x = msg.height / 2
        self.camera_center.y = msg.width / 2

    def marker_id_callback(self, msg):
        self.observed_marker_id = msg.data

    def marker_center_callback(self, msg):
        self.marker_position.x = msg.x
        self.marker_position.y = msg.y

        # Record new marker data if we're still gathering information
        if self.observed_marker_id and self.observed_marker_id not in self.collected_marker_ids:
            self.markers_location_map[self.observed_marker_id] = (msg.x, msg.y)

    def image_callback(self, msg):
        # Convert the compressed image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        holder_angle = Float64()

        if self.collection_phase:
            # Rotate until we have at least 7 markers identified
            if len(self.collected_marker_ids) < 7:
                self.current_rotation_angle += 0.07
                holder_angle.data = self.current_rotation_angle

                if self.observed_marker_id and self.observed_marker_id not in self.collected_marker_ids:
                    self.collected_marker_ids.append(self.observed_marker_id)
                    self.collected_marker_ids.sort()
                    rospy.loginfo(f"Collected marker IDs: {self.collected_marker_ids}")
            else:
                # Done collecting marker IDs
                self.collection_phase = False
                rospy.loginfo("All markers collected. Proceeding to alignment phase.")
        else:
            # Align camera with each marker in sequence
            if len(self.collected_marker_ids) > 0:
                self.active_marker = self.collected_marker_ids[0]
                target_x = self.marker_position.x
                target_y = self.marker_position.y
                rospy.loginfo(f"Active marker: {self.active_marker}, Target: ({target_x}, {target_y}), Camera Center: ({self.camera_center.x}, {self.camera_center.y})")

                # Check alignment
                if abs(self.camera_center.x - target_x) < 20 and self.observed_marker_id == self.active_marker:
                    self.alignment_achieved = True
                    holder_angle.data = self.current_rotation_angle
                    rospy.loginfo(f"Aligned with marker {self.active_marker}")

                    # Draw a circle around the aligned marker
                    cv2.circle(image_np, (int(target_x), int(target_y)), 20, (0, 255, 0), 3)

                    # Publish the processed image
                    image_msg = CompressedImage()
                    image_msg.header.stamp = rospy.Time.now()
                    image_msg.format = "jpeg"
                    image_msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tobytes()
                    self.processed_image_pub.publish(image_msg)

                if self.alignment_achieved:
                    # Move on to the next marker
                    self.collected_marker_ids.pop(0)
                    self.alignment_achieved = False
                    holder_angle.data = self.current_rotation_angle
                elif self.camera_center.x > target_x and self.observed_marker_id == self.active_marker:
                    # Need to rotate left
                    self.current_rotation_angle += 0.01
                    holder_angle.data = self.current_rotation_angle
                    rospy.loginfo("Rotating left to align with marker.")
                elif self.camera_center.x < target_x and self.observed_marker_id == self.active_marker:
                    # Need to rotate right
                    self.current_rotation_angle -= 0.01
                    holder_angle.data = self.current_rotation_angle
                    rospy.loginfo("Rotating right to align with marker.")
                else:
                    # Keep rotating until we find the target again
                    self.current_rotation_angle += 0.07
                    holder_angle.data = self.current_rotation_angle
                    rospy.loginfo("Continuing rotation to locate marker.")
            else:
                # All markers handled
                holder_angle.data = self.current_rotation_angle
                rospy.loginfo("All markers processed.")

        self.camera_holder_pub.publish(holder_angle)


def main():
    MarkerRotationController()
    rospy.spin()

if __name__ == '__main__':
    main()
