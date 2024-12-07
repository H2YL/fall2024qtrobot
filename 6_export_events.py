import datetime
import os.path
import pandas as pd
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError

SCOPES = ["https://www.googleapis.com/auth/calendar"]

def import_schedule():
    # Read the CSV file
    df = pd.read_csv('5_block_on_cal.csv')
    events = []

    # Convert each row into the desired format
    for _, row in df.iterrows():
        event = {
            'summary': row['Event name'],
            'start': {
                'dateTime': f"{row['startDate']}T{row['startTime']}+04:00",
                'timeZone': 'Asia/Dubai',
            },
            'end': {
                'dateTime': f"{row['endDate']}T{row['endTime']}+04:00",
                'timeZone': 'Asia/Dubai',
            },
        }
        events.append(event)

    return events

def get_existing_events(service):
    # Fetch existing events from the calendar
    now = datetime.datetime.utcnow().isoformat() + 'Z'  # 'Z' indicates UTC time
    events_result = service.events().list(calendarId='primary', timeMin=now,
                                          maxResults=100, singleEvents=True,
                                          orderBy='startTime').execute()
    return {event['summary']: event for event in events_result.get('items', [])}

def main():
    creds = None
    if os.path.exists("token.json"):
        creds = Credentials.from_authorized_user_file("token.json", SCOPES)

    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file("credentials.json", SCOPES)
            creds = flow.run_local_server(port=0)

        with open("token.json", "w") as token:
            token.write(creds.to_json())

    try:
        service = build("calendar", "v3", credentials=creds)
        events_to_add = import_schedule()
        
        existing_events = get_existing_events(service)

        for event in events_to_add:
            # Check if the event already exists in the calendar
            if event['summary'] not in existing_events:
                created_event = service.events().insert(calendarId='primary', body=event).execute()
                # print('Event created:', (created_event.get('htmlLink')))
            # else:
                # print(f"Event '{event['summary']}' already exists and will not be added.")

    except HttpError as error:
        print(f"An error occurred: {error}")

if __name__ == "__main__":
    main()