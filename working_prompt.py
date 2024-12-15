#!/usr/bin/env python3
from openai import OpenAI
from dotenv import load_dotenv
import os
import sys
import rospy
import re
import json
import csv

from qt_robot_interface.srv import behavior_talk_text
from std_msgs.msg import String
from qt_robot_interface.srv import *
from qt_gspeech_app.srv import *

# Load environment variables from .env (if you have a .env locally)
load_dotenv()

# Initialize OpenAI client with your API key
client = OpenAI(api_key="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")

# Wait for ROS services to be available
rospy.wait_for_service('/qt_robot/behavior/talkText')
talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
recognizeQuestion = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)

def load_tasks_from_csv(csv_path="2_prioritized_tasks.csv"):
    """
    Load tasks from a CSV file and format them into a string.
    Also identify the top priority (priority=1) task.
    CSV format: Priority,Task Name,Deadline
    """
    tasks = ""
    top_priority_task = None
    if not os.path.exists(csv_path):
        return tasks, top_priority_task

    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        headers = next(reader, None)  # skip the header row
        for row in reader:
            if len(row) >= 3:
                priority = row[0].strip()
                task_name = row[1].strip()
                deadline = row[2].strip()
                tasks += f"Priority {priority}: {task_name}, Deadline: {deadline}\n"
                if priority == "1" and top_priority_task is None:
                    top_priority_task = (task_name, deadline)
    return tasks.strip(), top_priority_task

task_list, top_priority_task = load_tasks_from_csv()
schedule = ""  # No schedule provided

def clean_text(input_text):
    """
    Remove extraneous punctuation and spaces.
    Also remove any {{END_MARKER}} tags from the final output.
    """
    # Remove any non-word characters except certain punctuation
    cleaned_text = re.sub(r"[^\w\s!,'?.:-]", "", input_text)
    # Normalize multiple spaces
    cleaned_text = re.sub(r"\s+", " ", cleaned_text).strip()
    # Remove any END_MARKER patterns
    end_marker_pattern = re.compile(r"\{\{?END_MARKER\}?\}", re.IGNORECASE)
    cleaned_text = end_marker_pattern.sub("", cleaned_text)
    return cleaned_text.strip()

def get_completion_from_messages(messages, model="gpt-4", temperature=0.7):
    """
    Calls OpenAI's chat completion API with the given messages.
    Adjust 'model' as needed (gpt-4, gpt-3.5-turbo, etc.).
    """
    response = client.chat.completions.create(
        model=model,
        messages=messages,
        temperature=temperature,
    )
    return response.choices[0].message.content

def start_conversation(detected_emotion):
    """
    Initiate a conversation with the user based on the detected_emotion.
    Alex (assistant) ends the conversation:
       - Immediately if END_MARKER is found in the assistant's message.
       - Or after 5 total messages from Alex (whichever comes first).
    """
    global task_list, top_priority_task, schedule

    # Map detected emotion to a more general feeling
    emotion_map = {
        "angry": "frustrated",
        "disgust": "upset",
        "fear": "anxious",
        "sad": "down",
        "surprise": "surprised",
        "happy": "happy",
        "neutral": "neutral"
    }
    general_emotion = emotion_map.get(detected_emotion.lower(), "upset")

    # Identify the top priority task
    if top_priority_task is not None:
        chosen_task_name, chosen_task_deadline = top_priority_task
        top_priority_str = f'Among these, they will work on their top-priority task: "{chosen_task_name}" (Deadline: {chosen_task_deadline}).'
    else:
        top_priority_str = "Among these, they have chosen a top-priority task to work on."

    # Initial assistant message
    initial_message = f"I noticed you might be feeling {general_emotion}. I'm here to listen if you'd like to talk about it."

    # System prompt with instructions
    system_prompt = f"""
You are Alex, a friendly and helpful robot assistant designed to support college students and young adults with ADHD. You also help them be more productive in their daily tasks. You are empathetic, patient, and understanding.

Here is the user's task list:
{task_list}

They will focus on their top-priority task: {top_priority_task}

You have started a 25-minute work session followed by a 5-minute break. 
You detected a negative emotion and want to console the user.

Rules:
1. Provide emotional support based on the user's responses.
2. Offer specific coping exercises if they remain stressed.
3. Encourage them to continue or resume their tasks.
4. If the user asks for the entire schedule or tasks, respond: "It is displayed for you on the app."
5. Limit your conversation to 5 total messages from Alex (the assistant). 
   On your 5th message, politely end the conversation with a final statement containing {{END_MARKER}}.
6. If at any time you decide the conversation should end earlier, also include {{END_MARKER}} and the conversation stops immediately.

End Condition:
- If user doesn't say anything new (blank), end with a final message containing {{END_MARKER}}.
- If Alex has already sent 5 messages, end the conversation.
- If Alex's message contains {{END_MARKER}}, we end the conversation immediately and do not print the marker itself.
- Example final message: "I'm glad I could help. Take care! {{END_MARKER}}"
"""

    context = [{'role': 'system', 'content': system_prompt}]

    # Print and speak the initial assistant message
    print(f"Alex: {initial_message}")
    talkText(initial_message)
    context.append({'role': 'assistant', 'content': initial_message})

    alex_message_count = 1  # We already used 1 assistant message

    while True:
        # If Alex has already sent 5 messages in total, forcibly end here
        if alex_message_count >= 5:
            forced_ending = "I think we've covered enough for now. Keep going with your task when you're ready. {{END_MARKER}}"
            print("Alex:", clean_text(forced_ending))
            talkText(clean_text(forced_ending))
            context.append({'role': 'assistant', 'content': forced_ending})
            print("Conversation ended gracefully after the 5th message.")
            break

        # Get user input from the terminal
        user_input = input("User: ").strip()
        if not user_input:
            # If user just pressed enter or typed nothing, end politely
            forced_ending = "It seems you have nothing more to add. I hope you feel better soon. {{END_MARKER}}"
            print("Alex:", clean_text(forced_ending))
            talkText(clean_text(forced_ending))
            context.append({'role': 'assistant', 'content': forced_ending})
            print("Conversation ended gracefully due to no user input.")
            break

        # Add user input to context
        context.append({'role': 'user', 'content': user_input})

        # Call OpenAI to get assistant's next response
        assistant_response = get_completion_from_messages(context)
        # Clean the response but don't remove the entire marker check logic
        cleaned_response = clean_text(assistant_response)

        alex_message_count += 1  # Alex is about to speak again

        # Check if there's an END_MARKER in the raw or cleaned response
        # to end conversation early if the model decided so.
        end_marker_pattern = re.compile(r"\{\{?END_MARKER\}?\}", re.IGNORECASE)
        if end_marker_pattern.search(assistant_response):
            # The raw response includes the marker, so conversation should end immediately
            # Remove the marker text from the final printed message
            final_text = end_marker_pattern.split(assistant_response)[0].strip()
            cleaned_final = clean_text(final_text)
            print("Alex:", cleaned_final)
            talkText(cleaned_final)
            context.append({'role': 'assistant', 'content': assistant_response})
            print("Conversation ended gracefully due to END_MARKER in response.")
            break
        else:
            # No end marker, just speak the response
            print("Alex:", cleaned_response)
            talkText(cleaned_response)
            context.append({'role': 'assistant', 'content': assistant_response})

    return

# Fix the python main guard
if __name__ == "__main__":
    # For testing, replace 'sad' with the actual detected emotion if needed
    detected_emotion = 'sad'  
    start_conversation(detected_emotion)
