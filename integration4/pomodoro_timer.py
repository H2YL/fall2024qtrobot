#!/usr/bin/env python3
# pomodoro_timer.py

import time
import threading
import rospy
from qt_robot_interface.srv import behavior_talk_text
from std_msgs.msg import String
import os
import fcntl
LOCK_FILE = "/tmp/conversation.lock"

class PomodoroTimer:
    def __init__(self):
        self.pomodoro_duration = 25 * 60  # Pomodoro duration in seconds
        self.short_break_duration = 5 * 60  # Short break duration in seconds
        self.long_break_duration = 2 * 60  # Long break duration in seconds
        self.remaining_time = self.pomodoro_duration
        self.timer_thread = None
        self.paused = False
        self.stop_event = threading.Event()
        self.state_pub = rospy.Publisher('/pomodoro_state', String, queue_size=10)
        rospy.wait_for_service('/qt_robot/behavior/talkText')
        self.talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
        self.start_engagement_pub = rospy.Publisher('start_engagement', String, queue_size=10)
        self.talking_pub = rospy.Publisher('stop_listening', String, queue_size=10)
        self.emotion_publisher = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
        self.session_count = 0  # To keep track of completed Pomodoro sessions
        self.total_sessions = 1  # Sessions before long break  # Changed from 4 to 1
        # Start a thread to monitor the lock file 
        self.lock_monitor_thread = threading.Thread(target=self.monitor_lock_file) 
        self.lock_monitor_thread.daemon = True 
        self.lock_monitor_thread.start() 
    
    def monitor_lock_file(self): 
        while not rospy.is_shutdown(): 
            if os.path.exists(LOCK_FILE): 
                lock_file = open(LOCK_FILE, "r+")
                fcntl.flock(lock_file, fcntl.LOCK_SH)
                lines = lock_file.readlines()
                lock_file.close()
               
                if any("locked by emotion detection." in line for line in lines): 
                    # print("are we there???")
                    if any("Releasing emotion lock in 2 minutes." in line for line in lines): 
                        if self.paused: # Resume if the lock is released 
                            rospy.loginfo("Lock released detected. Resuming Pomodoro timer.") 
                            self.resume() 
                    else: 
                        if not self.paused: # Pause if conversation is ongoing 
                            rospy.loginfo("Emotion Lock file detected. Pausing Pomodoro timer.") 
                            self.pause()
            # else: # No lock file; ensure the timer is not paused 
            #     if self.paused: 
            #         rospy.loginfo("Lock file removed. Resuming Pomodoro timer.") 
            #         self.resume() 
            time.sleep(1) # Check every second

    def talk(self, text):
        try:
            rospy.loginfo(f"Talking: {text}")
            self.talkText(text)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def start_pomodoro(self):
        rospy.loginfo("Starting Pomodoro session...")
        self.start_engagement_pub.publish('start')
        self.state_pub.publish('work')
        self.talking_pub.publish('stop')
        time.sleep(1)
        # if os.path.exists(LOCK_FILE):
        #     with open(LOCK_FILE, 'r') as f:
        #         for line in f:
        #             if line.strip() == "Releasing lock in 2 minutes.":
        #                 self.talk("Your work session has started.")
        #                 break
        # else:
        self.talk("Your work session has started.")
        time.sleep(1)
        self.talking_pub.publish('start')
        self.stop_event.clear()
        self.paused = False
        self.remaining_time = self.pomodoro_duration
        self.timer_thread = threading.Thread(
            target=self.run_timer,
            args=(self.pomodoro_duration, self.finish_pomodoro)
        )
        self.timer_thread.start()

    # def run_timer(self, duration, callback):
    #     flag = False
    #     short_break = False
    #     if flag == False and duration == self.short_break_duration:
    #         flag = True
    #         short_break = True

    #     if short_break == True:
    #         remaining_time = duration
    #         # print(remaining_time)
    #         while remaining_time > 0 and not self.stop_event.is_set():
    #             if self.paused:
    #                 time.sleep(1)
    #                 continue
                
    #             time.sleep(1)
    #             remaining_time -= 1
    #     else:

    #     if not self.stop_event.is_set():
    #         callback()

    def run_timer(self, duration, callback):
        flag = False
        short_break = False
        if flag == False and duration == self.short_break_duration:
            flag = True
            short_break = True

        if short_break == True:
            remaining_time = duration
            self.emotion_publisher.publish("break")
            while remaining_time > 0 and not self.stop_event.is_set():
                if self.paused:
                    # When paused, stop displaying the current emotion
                    time.sleep(1)
                    continue

                
                time.sleep(1)
                remaining_time -= 1

        else:
            remaining_time = duration
            self.current_emotion = None
            while remaining_time > 0 and not self.stop_event.is_set():
                if self.paused:
                    # When paused, stop displaying the current emotion
                    self.stop_displaying_emotion()
                    time.sleep(1)
                    continue

                # If not paused and no current emotion is being displayed, display the nearest video
                if self.current_emotion is None and not self.paused:
                    
                    nearest_time = self.find_nearest_video(remaining_time)
                    print(remaining_time, nearest_time)
                    self.display_emotion(nearest_time)  # Display the new emotion
                    self.current_emotion = nearest_time

                time.sleep(1)
                remaining_time -= 1

        if not self.stop_event.is_set():
            self.stop_displaying_emotion()
            callback()

    def find_nearest_video(self, remaining_time):
        video_duration = 10  

        # Round down the remaining time to the nearest video duration (floor division)
        nearest_time = (remaining_time // video_duration) * video_duration

        return nearest_time

    def display_emotion(self, remaining_time):
        # Publish to the robot's interface based on the remaining time
        video_name = f"work_videos/{remaining_time:04d}"  # Assuming videos are named based on remaining time
        print(video_name)
        rospy.loginfo(f"Time remaining: {video_name} seconds")
        self.emotion_publisher.publish(video_name)

    def stop_displaying_emotion(self):
        # Stop the current video from being displayed
        # if self.current_emotion is not None:
        
        # self.emotion_publisher.publish('stop')
        if self.current_emotion is not None:
            rospy.loginfo("Stopping current emotion video.")
            import subprocess
            subprocess.run('rosservice call /qt_robot/emotion/stop "{}"', shell=True, check=True)
        self.current_emotion = None  # Reset current emotion when paused

    def finish_pomodoro(self):
        self.talk("Your Pomodoro session has ended!")
        self.session_count += 1
        if self.session_count >= self.total_sessions:
            # self.long_break()  # Removed to prevent long break
            self.short_break()  # Changed to short_break instead of long_break
        else:
            self.short_break()

    def short_break(self):
        rospy.loginfo("Starting short break...")
        self.state_pub.publish('break')
        self.start_engagement_pub.publish('stop')
        self.talking_pub.publish('stop')
        time.sleep(1)
        # if os.path.exists(LOCK_FILE):
        #     with open(LOCK_FILE, 'r') as f:
        #         for line in f:
        #             if line.strip() == "Releasing lock in 2 minutes.":
        #                 self.talk("Your short break has started. You can relax for a while.")
        #                 break
        # else:
        self.talk("Your short break has started. You can relax for a while.")
        
        time.sleep(1)
        self.talking_pub.publish('start')
        self.stop_event.clear()
        self.paused = False
        self.timer_thread = threading.Thread(
            target=self.run_timer,
            args=(self.short_break_duration, self.stop_pomodoro)  # Changed callback from start_pomodoro to stop_pomodoro
        )
        self.timer_thread.start()

    def long_break(self):
        rospy.loginfo("Starting long break...")
        self.state_pub.publish('break')
        self.start_engagement_pub.publish('stop')
        self.talking_pub.publish('stop')
        time.sleep(1)
        self.talk("Great job! Your long break has started. You deserve a longer rest.")
        time.sleep(1)
        self.talking_pub.publish('start')
        self.stop_event.clear()
        self.paused = False
        self.timer_thread = threading.Thread(
            target=self.run_timer,
            args=(self.long_break_duration, self.finish_long_break)
        )
        self.timer_thread.start()

    def finish_long_break(self):
        self.talk("Your Pomodoro cycle has ended!")
        self.stop_pomodoro()

    def pause(self):
        rospy.loginfo("Pausing Pomodoro timer...")
        self.paused = True
        self.state_pub.publish('paused')

    def resume(self):
        rospy.loginfo("Resuming Pomodoro timer...")
        self.paused = False
        self.state_pub.publish('work')

    def stop_pomodoro(self):
        rospy.loginfo("Stopping Pomodoro session...")
        self.stop_event.set()
        self.paused = False
        self.state_pub.publish('pomodoro_completed')
        try:
            self.emotion_publisher.publish('stop')
            self.start_engagement_pub.publish('stop')
            self.talk("Your Pomodoro focus session has been stopped.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed during stop_pomodoro: {e}")

    def end_cycle(self):
        rospy.loginfo("Pomodoro cycle completed.")
        self.state_pub.publish('pomodoro_completed')
        self.start_engagement_pub.publish('stop')
        self.talking_pub.publish('stop')
        time.sleep(1)
        self.talk("Congratulations! You've completed all your Pomodoro sessions.")
        time.sleep(1)
        self.talking_pub.publish('start')

