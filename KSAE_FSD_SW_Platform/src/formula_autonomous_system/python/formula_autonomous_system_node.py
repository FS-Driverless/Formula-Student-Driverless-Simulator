#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file formula_autonomous_system_node.py
@author Jiwon Seok (jiwonseok@hanyang.ac.kr)
@brief Formula Student Driverless Autonomous System Node - Python Implementation
@version 0.1
@date 2025-07-25

@copyright Copyright (c) 2025

Note: This file is NOT modifiable by students. It contains the ROS node infrastructure.
"""

import rospy
import threading
import time
import numpy as np
from threading import Lock, Thread
from collections import deque

# ROS messages
from sensor_msgs.msg import PointCloud2, Image, Imu, NavSatFix
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
import tf2_ros
from tf2_geometry_msgs import tf2_geometry_msgs
import tf2_py
from rospy import *

# FS messages
from fs_msgs.msg import ControlCommand, FinishedSignal, GoSignal

# Image processing
import cv2
from cv_bridge import CvBridge

# Scientific computing
import sensor_msgs.point_cloud2 as pc2

# Import the main autonomous system - simplified approach
import sys
import os

# Add the source python directory to Python path
src_python_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'python')
if src_python_dir not in sys.path:
    sys.path.insert(0, src_python_dir)

# Now import from the source
from formula_autonomous_system import FormulaAutonomousSystem


class FormulaAutonomousSystemNode:
    """
    ROS Node wrapper for FormulaAutonomousSystem
    
    This class handles:
    - ROS message subscriptions and publications
    - Message synchronization and threading
    - System lifecycle management
    - Data visualization and debugging
    """
    
    def __init__(self):
        # System state
        self.is_initialized = False
        
        # Message containers
        self.lidar_msg = PointCloud2()
        self.camera1_msg = Image()
        self.camera2_msg = Image()
        self.imu_msg = Imu()
        self.gps_msg = NavSatFix()
        self.go_signal_msg = GoSignal()
        
        # Message initialization flags
        self.is_lidar_msg_init = False
        self.is_camera1_msg_init = False
        self.is_camera2_msg_init = False
        self.is_imu_msg_init = False
        self.is_gps_msg_init = False
        self.is_go_signal_msg_init = False
        
        # Message mutexes
        self.lidar_msg_mutex = Lock()
        self.camera1_msg_mutex = Lock()
        self.camera2_msg_mutex = Lock()
        self.imu_msg_mutex = Lock()
        self.gps_msg_mutex = Lock()
        self.go_signal_msg_mutex = Lock()
        
        # Output messages
        self.control_command_msg = ControlCommand()
        self.autonomous_mode_msg = String()
        
        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Threading
        self.main_thread = None
        self.main_loop_rate = 1000.0  # Hz
        self.count = 0

        # Algorithm system
        self.formula_autonomous_system = None
        
        # Initialize the system
        self.init()

    def init(self):
        """Initialize the ROS node and autonomous system"""
        try:
            # Initialize subscribers
            self.lidar_sub = rospy.Subscriber("/fsds/lidar/Lidar1", PointCloud2, self.lidar_callback, queue_size=1)
            self.camera1_sub = rospy.Subscriber("/fsds/cameracam1", Image, self.camera1_callback, queue_size=1)
            self.camera2_sub = rospy.Subscriber("/fsds/cameracam2", Image, self.camera2_callback, queue_size=1)
            self.imu_sub = rospy.Subscriber("/fsds/imu", Imu, self.imu_callback, queue_size=1)
            self.gps_sub = rospy.Subscriber("/fsds/gps", NavSatFix, self.gps_callback, queue_size=1)
            self.go_signal_sub = rospy.Subscriber("/fsds/signal/go", GoSignal, self.go_signal_callback, queue_size=1)
            
            # Initialize publishers
            self.control_pub = rospy.Publisher("/fsds/command_control", ControlCommand, queue_size=1)
            self.autonomous_mode_pub = rospy.Publisher("/fsds/AS_status", String, queue_size=1)

            # Initialize parameters
            self.main_loop_rate = rospy.get_param("/system/main_loop_rate", 1000.0)
            
            # Initialize autonomous system
            self.formula_autonomous_system = FormulaAutonomousSystem()
            if not self.formula_autonomous_system.init():
                rospy.logerr("Failed to initialize FormulaAutonomousSystem")
                return False
            
            self.is_initialized = True
            rospy.loginfo("FormulaAutonomousSystemNode: Initialization completed")
            return True
            
        except Exception as e:
            rospy.logerr(f"FormulaAutonomousSystemNode: Initialization failed: {e}")
            return False

    def get_parameters(self):
        """Get parameters from ROS parameter server"""
        # Parameters are handled by the FormulaAutonomousSystem class
        return True

    def run(self):
        """Main processing loop"""
        if not self.check_essential_messages():
            return
        
        # Get messages with thread safety
        with self.lidar_msg_mutex:
            lidar_msg = self.lidar_msg
        
        with self.camera1_msg_mutex:
            camera1_msg = self.camera1_msg
        
        with self.camera2_msg_mutex:
            camera2_msg = self.camera2_msg
        
        with self.imu_msg_mutex:
            imu_msg = self.imu_msg
        
        with self.gps_msg_mutex:
            gps_msg = self.gps_msg
        
        with self.go_signal_msg_mutex:
            go_signal_msg = self.go_signal_msg
        
        # Run autonomous system (Pythonic way)
        success, control_command, autonomous_mode = self.formula_autonomous_system.run(
            lidar_msg, camera1_msg, camera2_msg, imu_msg, gps_msg, go_signal_msg
        )
        
        if success:
            self.control_command_msg = control_command
            self.autonomous_mode_msg = autonomous_mode

        # Debug output
        self.count += 1
        if self.count % 100 == 0:
            rospy.loginfo(f"FormulaAutonomousSystemNode: Main loop rate: {self.main_loop_rate} Hz")
            self.count = 0

    def publish(self):
        """Publish all output messages"""
        self.publish_control()
        self.publish_autonomous_mode()

    def execute(self):
        """Start the main execution thread"""
        self.main_thread = Thread(target=self.main_loop)
        self.main_thread.daemon = True
        self.main_thread.start()

    def main_loop(self):
        """Main processing loop in separate thread"""
        rate = self.main_loop_rate  # Hz
        loop_rate = rospy.Rate(rate)
        
        while not rospy.is_shutdown():
            try:
                self.run()
                self.publish()
                loop_rate.sleep()
            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")
        
        rospy.loginfo("FormulaAutonomousSystemNode: Main thread terminated")

    def check_essential_messages(self):
        """Check if all essential messages have been received"""
        missing_messages = []
        
        if not self.is_lidar_msg_init:
            missing_messages.append("lidar")
        if not self.is_camera1_msg_init:
            missing_messages.append("camera1")
        if not self.is_camera2_msg_init:
            missing_messages.append("camera2")
        if not self.is_imu_msg_init:
            missing_messages.append("imu")
        if not self.is_gps_msg_init:
            missing_messages.append("gps")
        
        if missing_messages:
            rospy.logwarn_throttle(1.0, f"Waiting for messages: {', '.join(missing_messages)}")
            return False
        
        return True

    # ==================== ROS Callbacks ====================
    
    def lidar_callback(self, msg):
        """LiDAR point cloud callback"""
        with self.lidar_msg_mutex:
            self.lidar_msg = msg
            self.is_lidar_msg_init = True

    def camera1_callback(self, msg):
        """Camera 1 image callback"""
        with self.camera1_msg_mutex:
            self.camera1_msg = msg
            self.is_camera1_msg_init = True

    def camera2_callback(self, msg):
        """Camera 2 image callback"""
        with self.camera2_msg_mutex:
            self.camera2_msg = msg
            self.is_camera2_msg_init = True

    def imu_callback(self, msg):
        """IMU data callback"""
        with self.imu_msg_mutex:
            self.imu_msg = msg
            self.is_imu_msg_init = True

    def gps_callback(self, msg):
        """GPS data callback"""
        with self.gps_msg_mutex:
            self.gps_msg = msg
            self.is_gps_msg_init = True

    def go_signal_callback(self, msg):
        """Go signal callback"""
        with self.go_signal_msg_mutex:
            self.go_signal_msg = msg
            self.is_go_signal_msg_init = True

    # ==================== Publishers ====================
    
    def publish_control(self):
        """Publish control commands"""
        self.control_pub.publish(self.control_command_msg)

    def publish_autonomous_mode(self):
        """Publish autonomous mode status"""
        self.autonomous_mode_pub.publish(self.autonomous_mode_msg)


def main():
    """Main function"""
    try:
        # Initialize ROS
        rospy.init_node('formula_autonomous_system_python', anonymous=True)
        
        # Create node instance
        node = FormulaAutonomousSystemNode()
        
        # Start execution
        node.execute()
        
        # Keep node running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("FormulaAutonomousSystemNode: Interrupted by user")
    except Exception as e:
        rospy.logerr(f"FormulaAutonomousSystemNode: Fatal error: {e}")
    finally:
        rospy.loginfo("FormulaAutonomousSystemNode: Shutting down")


if __name__ == "__main__":
    main()