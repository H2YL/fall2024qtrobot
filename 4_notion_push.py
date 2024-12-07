import base64
import json
import os
import requests
import urllib.parse
from notion_client import Client
import datetime
import csv

# Notion API Authentication
oauth_client_id = "12ed872b-594c-8085-9ec3-00372e156eb1"
oauth_client_secret = "secret_yo5Gn7B36047cAvlhEj6RD6tiAd4TQ59RXEzbu0pcq4"
redirect_uri = "https://nyuad.nyu.edu/en/"

# Generate authentication URL
parsed_redirect_uri = urllib.parse.quote_plus(redirect_uri)
auth_url = f"https://api.notion.com/v1/oauth/authorize?client_id={oauth_client_id}&response_type=code&owner=user&redirect_uri={parsed_redirect_uri}"

# Manually obtain the auth code from the redirect URL
auth_code = "035b473c-be50-44c9-a775-fc6595b7381d"

# Encode client ID and secret
key_secret = f'{oauth_client_id}:{oauth_client_secret}'.encode('ascii')
b64_encoded_key = base64.b64encode(key_secret).decode('ascii')

# Get access token
base_url = 'https://api.notion.com/v1/oauth/token'
auth_headers = {
    'Authorization': f'Basic {b64_encoded_key}',
    'Content-Type': 'application/x-www-form-urlencoded;charset=UTF-8',
}

auth_data = {
    'grant_type': 'authorization_code',
    'code': auth_code,
    'redirect_uri': redirect_uri,
}

auth_resp = requests.post(base_url, headers=auth_headers, data=auth_data)
# access_token = auth_resp.json()['access_token']
access_token = "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"

# Initialize Notion client
notion = Client(auth=access_token)

def process_block(block, notion):
    """Recursively processes a block and its children to find to-do lists."""
    if block['has_children']:
        children = notion.blocks.children.list(block['id'])['results']
        for child in children:
            if child['type'] == 'to_do' and len(child['to_do']['rich_text']) > 0:
                print("task: ", child['to_do']['rich_text'][0]['text']['content'])
                print("finished: ", child['to_do']['checked'])
                print("=============================================")
            if child['has_children']:
                process_block(child, notion)

def tasks_for_the_day(b):
    today = datetime.datetime.now().strftime("%a")
    for block in b['results']:
        if block['type'] == 'column_list' and block['has_children']:
            children = notion.blocks.children.list(block['id'])['results']
            for child in children:
                if child['has_children']:
                    children_children = notion.blocks.children.list(child['id'])['results']
                    day = children_children[0]['heading_3']['rich_text'][0]['text']['content']
                    if today in day:
                        return children_children[1:]
    return []

def find_task_to_break(t, all_tasks_for_the_day):
    for task in all_tasks_for_the_day:
        if task['type'] == 'to_do' and task['to_do']['rich_text']:
            task_name = task['to_do']['rich_text'][0]['plain_text']
            if t in task_name:
                return task
    return None

def read_weekly_tasks(file_path):
    tasks = {}
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header row
        for row in reader:
            day, task = row
            if day not in tasks:
                tasks[day] = []
            tasks[day].append(task)
    return tasks

def write_tasks_to_notion(notion, page_id, tasks):
    for day, day_tasks in tasks.items():
        # Create a header for each day
        notion.blocks.children.append(
            block_id=page_id,
            children=[{
                "object": "block",
                "type": "heading_3",
                "heading_3": {
                    "rich_text": [{"type": "text", "text": {"content": day}}]
                }
            }]
        )
        
        # Add tasks for the day
        for task in day_tasks:
            notion.blocks.children.append(
                block_id=page_id,
                children=[{
                    "object": "block",
                    "type": "to_do",
                    "to_do": {
                        "rich_text": [{"type": "text", "text": {"content": task}}],
                        "checked": False
                    }
                }]
            )
    
    print("All tasks have been added to Notion.")

def main():
    # Search for to-do list
    results = notion.search(query="to-do list").get("results")[0]
    page_id = results["id"]
    b = notion.blocks.children.list(page_id)

    # Read tasks from CSV
    tasks = read_weekly_tasks('3_weekly_tasks.csv')
    
    # Write tasks to Notion
    write_tasks_to_notion(notion, page_id, tasks)

if __name__ == "__main__":
    main()