#!/usr/bin/env python3
import rospy
import time
from emotion_detection_array import EmotionDetector
from qt_robot_interface.srv import behavior_talk_text
from std_msgs.msg import String
import threading
import os
import fcntl
from working_prompt import start_conversation

LOCK_FILE = "/tmp/conversation.lock"

def acquire_lock():
    try:
        if os.path.exists(LOCK_FILE):
            return False
        else:
            lock_file = open(LOCK_FILE, "w+")
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            lock_file.write("locked by emotion detection.\n")
            lock_file.flush()
            lock_file.close()
            return True
    except IOError:
        return False
    except Exception as e:
        print(f"An error occurred: {e}")
        return False

def release_lock():
    try:
        lock_file = open(LOCK_FILE, "r+")
        fcntl.flock(lock_file, fcntl.LOCK_UN)
        lock_file.close()
        print("Lock released.")
        os.remove(LOCK_FILE)
    except Exception as e:
        print(f"Error releasing lock: {e}")

def release_lock_with_delay(delay_seconds):
    def delay_release():
        lock_file = open(LOCK_FILE, "a")
        lock_file.write("Releasing emotion lock in 2 minutes.")
        lock_file.flush()
        lock_file.close()
        time.sleep(delay_seconds)
        release_lock()
    threading.Thread(target=delay_release, daemon=True).start()

class EmotionMonitor:
    def __init__(self):
        print("Starting Emotion Monitor...")
        # rospy.init_node('emotion_monitor_node')
        self.emotion_detector = EmotionDetector()
        self.talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
        self.emotion_publisher = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)

        self.is_conversation_active = False
        self.last_conversation_time = 0
        self.conversation_cooldown = 120

        self.analysis_interval = 30
        self.emotion_detector.max_history_size = int(self.analysis_interval / self.emotion_detector.detection_interval_sec)

        self.last_check_time = time.time()

    def run(self):
        try:
            while not rospy.is_shutdown():
                frame, emotion = self.emotion_detector.detect_emotion()
                if emotion is not None:
                    print("Detected Emotion: ", emotion)

                if time.time() - self.last_check_time >= self.analysis_interval:
                    self.check_emotion_threshold()
                    self.last_check_time = time.time()

                rospy.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.emotion_detector.release_resources()

    def is_negative_emotion(self, emotion):
        negative_emotions = ['angry', 'disgust', 'fear', 'sad', 'surprise']
        return emotion.lower() in negative_emotions

    def check_emotion_threshold(self):
        if not self.emotion_detector.emotion_history:
            return

        negative_count = sum(1 for emotion in self.emotion_detector.emotion_history if self.is_negative_emotion(emotion))
        total_count = len(self.emotion_detector.emotion_history)
        negative_percentage = (negative_count / total_count) * 100

        self.emotion_detector.emotion_history.clear()

        threshold = 80  
        if negative_percentage >= threshold and not self.is_conversation_active:
            self.handle_negative_emotion_detected()

    def handle_negative_emotion_detected(self, emotion=None):
        current_time = time.time()
        if self.is_conversation_active:
            return
        if current_time - self.last_conversation_time < self.conversation_cooldown:
            return

        self.is_conversation_active = True
        if not emotion:
            emotion = next((e for e in reversed(self.emotion_detector.emotion_history) if self.is_negative_emotion(e)), "sad")

        rospy.loginfo(f"Detected negative emotion: {emotion}")
        if not acquire_lock():
            print("Conversation locked by another script.")
        else:    
            user_feels_better = start_conversation(emotion)  # Uses working_prompt with input()
            if user_feels_better:
                rospy.loginfo("User feels better now.")
            else:
                rospy.loginfo("Conversation ended.")
            release_lock_with_delay(100)
            print("Lock will be released in 2 minutes.")
            self.last_conversation_time = current_time
        self.is_conversation_active = False

if __name__ == "__main__":
    monitor = EmotionMonitor()
    monitor.run()
