#!/usr/bin/env python3
import rospy
import threading
import time

from engagement_main import MainApp
from emotion_main import EmotionMonitor

if __name__ == "__main__":
    # Initialize a ROS node for the combined application
    rospy.init_node('combined_app_node')

    # Create instances of both main applications
    engagement_app = MainApp()
    emotion_monitor = EmotionMonitor()

    # Run each in a separate thread
    engagement_thread = threading.Thread(target=engagement_app.run)
    emotion_thread = threading.Thread(target=emotion_monitor.run)

    engagement_thread.start()
    emotion_thread.start()

    # Keep the main thread alive until shutdown
    try:
        while not rospy.is_shutdown():
            time.sleep(1)
    except KeyboardInterrupt:
        pass
