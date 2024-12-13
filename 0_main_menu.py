import subprocess
import csv
import getpass
import sys
import os
from datetime import datetime, timedelta

def print_divider():
    print("==========================================")

def print_intro():
    print("Welcome! I am QTrobot, a humanoid social robot assistant designed to support college students and young adults with ADHD.")
    print("I am a productivity coach bot that can help you prioritize tasks and generate a schedule.")

def login():
    print("=====================login=====================")
    username = input("Enter your username: ")
    password = getpass.getpass("Enter your password: ")

    with open("0_userinfo.csv", "r") as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            if len(row) >= 2 and row[0] == username and row[1] == password:
                print("Login successful!")
                return True
    
    print("Invalid username or password. Please try again.")
    return False

def signup():
    print("=====================Sign Up=====================")
    while True:
        username = input("Enter a username (4-25 characters): ")
        if 4 <= len(username) <= 25:
            if not username_exists(username):
                break
            else:
                print("Error: Username already exists. Please choose a different username.")
        else:
            print("Error: Username must be between 4 and 25 characters. Please try again.")

    while True:
        password = getpass.getpass("Enter a password (4-25 characters): ")
        if 4 <= len(password) <= 25:
            break
        else:
            print("Error: Password must be between 4 and 25 characters. Please try again.")

    with open("0_userinfo.csv", "a", newline='') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow([username, password])
    
    print("Sign up successful! Log in with the registered information.")

def username_exists(username):
    with open("0_userinfo.csv", "r") as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            if row[0] == username:
                return True
    return False

def get_yesterday_tasks():
    yesterday = (datetime.now() - timedelta(days=1)).strftime('%a, %b %d')
    yesterday_tasks = []
    with open("3_weekly_tasks.csv", 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header
        for row in reader:
            if row[0] == yesterday:
                yesterday_tasks.append(row[1])
                print(row)
    return yesterday_tasks

def check_returning_user():
    files_to_check = [
        "1_calendar_items.csv",
        "2_prioritized_tasks.csv",
        "3_weekly_tasks.csv"
    ]
    
    completed_steps = 0
    
    for i, file in enumerate(files_to_check, start=1):
        if os.path.exists(file):
            completed_steps = i
        else:
            break

    if os.path.exists("5_block_on_cal.csv"):
        completed_steps = 5
    
    if completed_steps == 0:
        print("Welcome, new user! Let's get started with importing your schedule from Google Calendar.")
    elif completed_steps == 1:
        print("Welcome back! Let's prioritize your tasks.")
    elif completed_steps == 2:
        print("Welcome back! Let's create a weekly task list.")
    elif completed_steps == 3:
        print("Welcome back! Let's export your weekly task list to notion or create a daily schedule.")
    elif completed_steps == 5:
        if (len(get_yesterday_tasks()) == 0):
            print("Welcome! It looks like you are a new user or have not used the program in the last 7 days, let's get started with importing your schedule from Google Calendar.")
        else:
            print("Welcome back! Tell me how your study session went yesterday.")
            subprocess.run(['python', '7_returning_user.py'])

    print_divider()

def print_menu():
    print("Please enter the number to select the task that you want to execute:")
    print("1. Import your current schedule from Google Calendar")
    print("2. Converse with me to prioritize tasks")
    print("3. Converse with me to assign tasks for the next 7 days")
    print("4. Export the weekly task breakdown to Notion")
    print("5. Converse with me to schedule the tasks assigned to today")
    print("6. Export the generated daily schedule to Google Calendar")
    print("7. Start a focus session")
    print("Enter 'exit' to quit")

def execute_menu(choice):
    if choice == '1':
        subprocess.run(['python', '1_fetch_events.py'])
    elif choice == '2':
        subprocess.run(['python', '2_task_prioritization.py'])
    elif choice == '3':
        if os.path.exists("2_prioritized_tasks.csv"):
            subprocess.run(['python', '3_weekly_schedule.py'])
        else:
            print("You must complete step 2 before this step.")
    elif choice == '4':
        if os.path.exists("3_weekly_tasks.csv"):
            subprocess.run(['python', '4_notion_push.py'])
        else:
            print("You must complete step 3 before this step.")
    elif choice == '5':
        if os.path.exists("3_weekly_tasks.csv"):
            subprocess.run(['python', '5_create_schedule.py'])
        else:
            print("You must complete step 3 before this step.")
    elif choice == '6':
        if os.path.exists("5_block_on_cal.csv"):
            subprocess.run(['python', '6_export_events.py'])
            print("Would you like to start a focus session?")
        else:
            print("You must complete step 5 before this step.")
    elif choice == '7':
        if os.path.exists("2_prioritized_tasks.csv"):
            #run Mira's code
            original_dir = os.getcwd()
            os.chdir("focus_session")
            subprocess.run(['python', 'combined_main.py'])
            os.chdir(original_dir)
        else:
            print("You must complete step 2 before this step.")
    elif choice.lower() == 'menu':
        print_menu()
    elif choice.lower() == 'exit':
        return False
    else:
        print("Invalid choice. Please try again. Enter 'menu' to see the options.")
    return True

def main():
    print_intro()
    logged_in = False
    while (logged_in == False):
        print("Press 1 to log in")
        print("Press 2 to sign up")
        choice = input("Enter your choice: ")
        if (choice == '1'):
            logged_in = login()
        elif (choice == '2'):
            signup()
        else: 
            print("Invalid choice. Please try again.")
        print_divider()

    check_returning_user()

    print_menu()

    continue_program = True
    while continue_program:
        print("Enter 'menu' to see the options again")
        choice = input("Enter your choice: ")
        print_divider()
        continue_program = execute_menu(choice)
        print_divider()

if __name__ == "__main__":
    main()