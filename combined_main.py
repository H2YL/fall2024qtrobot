#!/usr/bin/env python3
import rospy
import threading
import time
import sys
import re

from engagement_main import MainApp
from emotion_main import EmotionMonitor

class TerminalFilter:
    """
    A custom stdout filter that only prints lines containing "Alex:" line-by-line.
    Lines containing "User:" won't appear in the console at all,
    allowing you to type user input on the same line (e.g., input("User: ")).
    """
    def __init__(self, original_stdout):
        self._original_stdout = original_stdout
        self.keep_pattern = re.compile(r'(Alex:)', re.IGNORECASE)
        self.buffer = ""

    def write(self, text):
        # Accumulate text until we encounter newline(s)
        self.buffer += text
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            # If line matches the pattern (i.e., has "Alex:"), print it
            if self.keep_pattern.search(line):
                self._original_stdout.write(line + "\n")

    def flush(self):
        # If there's any leftover text without a trailing newline
        if self.buffer:
            # Check if it matches the pattern
            if self.keep_pattern.search(self.buffer):
                self._original_stdout.write(self.buffer + "\n")
            self.buffer = ""
        self._original_stdout.flush()

# Instantiate and assign our filter to sys.stdout
sys.stdout = TerminalFilter(sys.stdout)

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