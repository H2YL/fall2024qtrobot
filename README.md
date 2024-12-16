# Leveraging Social Robots to Address Productivity Challenges in College Students with ADHD
Attention-Deficit Hyperactivity Disorder (ADHD) significantly impacts the academic, social, emotional, and psychological well-being of young adults. While various treatments and medications are available, there remains a notable absence of practical tools tailored to help them navigate their daily challenges. To bridge this gap, our research centers on designing software solutions that integrate seamlessly with the QTrobot, developed by LuxAI. These tools are specifically designed to support students in managing and completing tasks, and they are divided into the following key modules:
- Conversation
- Prioritization and Schedule Generation
- Pomodoro Session
- Engagement Detection 
- Emotion Recognition

## Instructions to Run

### Prerequisites

Before running the code, ensure the following software and libraries are installed on your system:

1. **DeepFace**: Required for emotion detection.
2. **Engagement Detection Model**: Used to analyze engagement levels.

## Setup Instructions

### Step 1: Install DeepFace

DeepFace is used for the emotion detection module. To install it, run the following command in your terminal:

```bash
pip install deepface
```

For more information, visit the [DeepFace documentation](https://pypi.org/project/deepface/).

### Step 2: Download the Engagement Detection Model

Clone the engagement detection repository from GitHub to access the required model files. Run the following command in your terminal:

```bash
git clone https://github.com/LCAS/engagement_detector.git
```

For additional details, refer to the [repository documentation](https://github.com/LCAS/engagement_detector?tab=readme-ov-file).

### Step 3: Set Up the Server

The server script (`server.py`) manages communication between different modules. To start the server, run the following command in a terminal:

```bash
python3 server.py
```

### Step 4: Run the Main Menu Script

In a separate terminal, navigate to the project directory and run the main menu script (`0_main_menu.py`) to begin interacting with the system:

```bash
python3 0_main_menu.py
```

## Additional Notes

- Make sure to configure any additional dependencies or environment variables as required by your system.
- To start a new user session, run ```make clean``` to delete all csv files and the token.json file
- For faster testing, the sample files can be copy+pasted from sample_files directory if you want to skip some steps
