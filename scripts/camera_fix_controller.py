#!/usr/bin/env python3

import rospy
import time
import math
import sys
import numpy as np
from scipy.ndimage import filters
import imutils
import cv2

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist, Point 
from sensor_msgs.msg import CompressedImage, CameraInfo

class MarkerTracker:

    def __init__(self):
        rospy.init_node('marker_tracking_node')

        self.camera_center = Point()
        self.marker_position = Point()
        
        self.tracked_marker_ids = []
        self.marker_positions = {}
        
        self.current_marker_id = 0
        self.collecting_marker_ids = True  
        self.aligned_with_marker = False
        self.active_marker = 0
        
        # Publishers
        self.processed_image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1)
        rospy.Subscriber("/robot/camera1/camera_info", CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber("/marker/id_number", Int32, self.marker_id_callback, queue_size=1)
        rospy.Subscriber("/marker/center_loc", Point, self.marker_center_callback, queue_size=1)
        
    def camera_info_callback(self, msg):
        # Determine the camera center from its resolution
        self.camera_center.x = msg.height / 2
        self.camera_center.y = msg.width / 2

    def marker_id_callback(self, msg):
        self.current_marker_id = msg.data

    def marker_center_callback(self, msg):
        self.marker_position.x = msg.x
        self.marker_position.y = msg.y
        
        # If we have a new marker ID that isn't in the list, store its center
        if self.current_marker_id and self.current_marker_id not in self.tracked_marker_ids:
            self.marker_positions[self.current_marker_id] = (msg.x, msg.y)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        vel = Twist()
        
        if self.collecting_marker_ids:
            # Rotate the robot to gather marker IDs until we have at least 7
            if len(self.tracked_marker_ids) < 7:
                vel.linear.x = 0
                vel.angular.z = 0.7

                if self.current_marker_id and self.current_marker_id not in self.tracked_marker_ids:
                    self.tracked_marker_ids.append(self.current_marker_id)
                    self.tracked_marker_ids.sort()
                    rospy.loginfo(f"Updated marker IDs: {self.tracked_marker_ids}")

            else:
                # After collecting all markers, move to the aligning/drawing phase
                self.collecting_marker_ids = False
                rospy.loginfo("All marker IDs collected. Proceeding to alignment phase.")

        else:
            # We have all markers. Now align the camera with each marker in order.
            if len(self.tracked_marker_ids) > 0:
                self.active_marker = self.tracked_marker_ids[0]
                target_x = self.marker_position.x
                target_y = self.marker_position.y
                rospy.loginfo(f"Target marker: {self.active_marker}, Target Pos: ({target_x}, {target_y}), Camera Center: ({self.camera_center.x}, {self.camera_center.y})")

                # Check if aligned with the active marker
                if abs(self.camera_center.x - target_x) < 10 and self.current_marker_id == self.active_marker:
                    self.aligned_with_marker = True
                    vel.angular.z = 0
                    rospy.loginfo(f"Aligned with marker {self.active_marker}. Marking on image.")

                    # Draw a circle on the target marker position
                    cv2.circle(image_np, (int(target_x), int(target_y)), 20, (0, 255, 0), 3)

                    Image_msg = CompressedImage()
                    Image_msg.header.stamp = rospy.Time.now()
                    Image_msg.format = "jpeg"
                    Image_msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tobytes()
                    
                    self.processed_image_pub.publish(Image_msg)

                if self.aligned_with_marker:
                    # Once aligned, remove it from the list and reset
                    self.tracked_marker_ids.pop(0)
                    self.aligned_with_marker = False
                    vel.angular.z = 0

                elif self.camera_center.x > target_x and self.current_marker_id == self.active_marker:
                    # Turn left
                    vel.angular.z = 0.3
                    rospy.loginfo("Turning left to align with marker.")

                elif self.camera_center.x < target_x and self.current_marker_id == self.active_marker:
                    # Turn right
                    vel.angular.z = -0.3
                    rospy.loginfo("Turning right to align with marker.")
                else:
                    # If no marker is identified or conditions not met, slowly turn
                    vel.angular.z = 0.5
                    rospy.loginfo("Searching for marker alignment...")

            else:
                # No more markers to find
                vel.angular.z = 0
                rospy.loginfo("All markers have been aligned.")

        self.cmd_vel_pub.publish(vel)


def main():
    MarkerTracker()
    rospy.spin()
    
    
if __name__ == '__main__':
    main()
