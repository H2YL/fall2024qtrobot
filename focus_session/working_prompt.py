
from openai import OpenAI
from dotenv import load_dotenv
import os
import sys
import rospy
from qt_robot_interface.srv import behavior_talk_text
from std_msgs.msg import String
from qt_robot_interface.srv import *
from qt_gspeech_app.srv import *
# from riva_speech_recognition_vad import RivaSpeechRecognitionSilero
import re
import json
import csv

# Load environment variables
load_dotenv()  # Read local .env file

# Initialize OpenAI client with your API key
client = OpenAI(api_key="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")

# Wait for the talkText service to be available
rospy.wait_for_service('/qt_robot/behavior/talkText')
talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
recognizeQuestion = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)

def load_tasks_from_csv(csv_path="2_prioritized_tasks.csv"):
    """
    Load tasks from the given CSV file and format them into a string.
    Also identify the top priority (priority 1) task.
    CSV format: Priority,Task Name,Deadline
    """
    tasks = ""
    top_priority_task = None
    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        headers = next(reader, None)  # Skip the header row
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
schedule = ""  # No schedule provided, leave it empty


def clean_text(input_text):
    """
    Clean the text by removing emojis, unnecessary punctuation, and extra spaces.
    """
    emoji_pattern = re.compile(
        "["
        "\U0001F600-\U0001F64F"  # Emoticons
        "\U0001F300-\U0001F5FF"  # Symbols & Pictographs
        "\U0001F680-\U0001F6FF"  # Transport & Map Symbols
        "\U0001F1E0-\U0001F1FF"  # Flags
        "\U00002702-\U000027B0"  # Dingbats
        "\U000024C2-\U0001F251"  # Enclosed Characters
        "]+",
        flags=re.UNICODE,
    )
    text_without_emojis = emoji_pattern.sub("", input_text)
    cleaned_text = re.sub(r"[^\w\s!,'?.:-]", "", text_without_emojis)
    cleaned_text = re.sub(r"\s+", " ", cleaned_text).strip()

    # Remove any END_MARKER if present
    cleaned_text = re.sub(r"\{\{?END_MARKER\}?\}", "", cleaned_text, flags=re.IGNORECASE).strip()
    return cleaned_text



def get_completion_from_messages(messages, model="gpt-4o", temperature=0.7):
    response = client.chat.completions.create(
        model=model,
        messages=messages,
        temperature=temperature,
    )
    return response.choices[0].message.content

def start_conversation(detected_emotion):
    # Map the detected emotion to a general feeling
    global task_list
    global schedule
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

    # Identify the top priority task (already loaded above)
    chosen_task_str = ""
    if top_priority_task is not None:
        chosen_task_name, chosen_task_deadline = top_priority_task
        top_priority_str = f'Among these, they will work on their top-priority task: "{chosen_task_name}" (Deadline: {chosen_task_deadline}).'
    else:
        top_priority_str = "Among these, they have chosen a top-priority task to work on."

    # Initial message from the assistant
    initial_message = f"I noticed you might be feeling {general_emotion}. I'm here to listen if you'd like to talk about it."

    # Conversation context with improved system prompt
    global context
    context = [
    {'role': 'system', 'content': f"""
You are Alex, a friendly and helpful robot assistant designed to support college students and young adults with ADHD. You also help them be more productive in their daily tasks. You are empathetic, patient, and understanding.
Before the focus session began, you helped the student by asking for all their tasks and assisting them in breaking them down into sub-tasks and prioritizing them. Here are all the tasks they shared, prioritized from most important to least important with their deadlines:{task_list}
Among these, they will work on their top-priority task: "{chosen_task_name}" (Deadline: {chosen_task_deadline}). They will focus on this task for 30 minutes.
Now, you have asked them to work on one of the top-three most important tasks based on the priorities assigned earlier in the focus session. They choose the task themselves from the top-three most important tasks. They have to work on the task for 30 minutes.  
You have started a pomodoro timer for them with 25 minutes of work followed by 5 minutes of break. You are monitoring if they are stressed. Always maintain this role and never break character.

You have detected a negative emotion. Your primary tasks now are:

1. **Emotional Support:**
   - If the user expresses negative emotions, respond with empathy.
   - Ask open-ended questions to encourage them to share more about their feelings.
   - Provide personalized advice or coping mechanisms relevant to their situation.
   - Offer the following specific exercises when appropriate:
     - **Breathing Exercise:**
       - Step 1: Breathe in slowly, counting to four.
       - Step 2: Hold your breath for four seconds.
       - Step 3: Slowly exhale through your mouth for four seconds.
       - Step 4: Repeat these steps a few times until you feel re-centered.
     - **5,4,3,2,1 Grounding Exercise:**
       - Ask the user to name:
         - 5 things they can see,
         - 4 things they can touch,
         - 3 things they can hear,
         - 2 things they can smell,
         - 1 thing they can taste.
       - Encourage them to take their time with each step to help shift focus to the present moment.
   - Suggest other coping strategies, such as:
     - Mindfulness and meditation exercises.
     - Physical activities like stretching or a short walk.
     - Time management and study techniques.
     - Creative outlets like drawing or writing.
     - Recommending professional help if appropriate.

2. **Specific Assistance:**
   - If the user asks for help with specific tasks, offer practical advice and tips.
   - Make sure to keep checking on their emotional and mental state even if you are providing tips.

3. **Positive Reinforcement:**
   - Celebrate the user's successes and efforts.
   - Encourage them to recognize their strengths and achievements.

4. **Conversation Termination:**
   - Once the user indicates they have nothing more on their mind, respond with a closing message and include the marker `{{END_MARKER}}` at the end of your response. For example:
     - "I'm glad I could help. If you need anything else, feel free to reach out. Take care! {{END_MARKER}}"

**Communication Guidelines:**
- If the user asks for their schedule or their task list, please say "It is displayed for you on the app." Please do not read out the entire schedule or the entire task list.
- **Tone:**
  - Use a warm, understanding, and patient tone.
  - Be supportive and non-judgmental.

- **Style:**
  - Keep responses concise and clear.
  - Use simple language that is easy to understand.
  - When guiding through an exercise, list each step on a new line with bullets or numbers for clarity.

- **Do Not:**
  - Do not provide medical diagnoses.
  - Do not mention being an AI language model or assistant.
  - Do not break character or the fourth wall.

- **Always:**
  - Personalize your responses based on the user's input.
  - Encourage the user to share their thoughts and feelings.
  - Be proactive in offering help but respect the user's boundaries if they decline.

Remember, your goal is to make the user feel heard, supported, and empowered to take positive steps forward.
"""}
]


    print(f"Alex: {initial_message}")
    talkText(initial_message)
    context.append({'role': 'assistant', 'content': initial_message})

    while True:
        # Get user input from the terminal
        user_input = input("User: ").strip()

        # Add user input to context
        context.append({'role': 'user', 'content': user_input})

        # Get assistant's response
        assistant_response = get_completion_from_messages(context)
        cleaned_response = clean_text(assistant_response)

        # Check if {END_MARKER} is in the response
        
        # Check if {END_MARKER} is in the response (case-insensitive)
        end_marker_pattern = re.compile(r"\{\{?END_MARKER\}?\}", re.IGNORECASE)
        if end_marker_pattern.search(cleaned_response):
            # Extract the response before the marker
            response_to_user = end_marker_pattern.split(cleaned_response)[0].strip()
            if response_to_user:
                print("Alex:", response_to_user)
                talkText(response_to_user)
                context.append({'role': 'assistant', 'content': response_to_user})
            # Optionally, send a final message or perform cleanup here
            print("Conversation ended gracefully.")
            break

        # If the response was cleaned, use the cleaned version
        if assistant_response != cleaned_response:
            print("Cleaned:", cleaned_response)
            talkText(cleaned_response)
            context.append({'role': 'assistant', 'content': cleaned_response})
        else:
            print("Alex:", assistant_response)
            talkText(assistant_response)
            context.append({'role': 'assistant', 'content': assistant_response})

    return

if __name__ == "__main__":
    # For testing purposes, replace 'sad' with the detected emotion
    detected_emotion = 'sad'  # Replace with the detected emotion
    start_conversation(detected_emotion)

