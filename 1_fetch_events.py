import datetime
import os.path
import csv
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError

# If modifying these scopes, delete the file token.json.
SCOPES = ["https://www.googleapis.com/auth/calendar"]

def main():
    """Fetches events from the user's primary Google Calendar and exports them to a CSV file."""
    creds = None
    
    if os.path.exists("token.json"):
        creds = Credentials.from_authorized_user_file("token.json", SCOPES)
    
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(
                "credentials.json", SCOPES
            )
            creds = flow.run_local_server(port=0)
        with open("token.json", "w") as token:
            token.write(creds.to_json())

    try:
        service = build("calendar", "v3", credentials=creds)

        # Calculate the time range
        now = datetime.datetime.utcnow()
        seven_days_later = now + datetime.timedelta(days=7)
        now = now.isoformat() + "Z"  # 'Z' indicates UTC time
        seven_days_later = seven_days_later.isoformat() + "Z"

        events_result = service.events().list(
            calendarId='primary',
            timeMin=now,
            timeMax=seven_days_later,
            singleEvents=True,
            orderBy='startTime'
        ).execute()

        events = events_result.get('items', [])
        filtered_events = [
            event for event in events 
            if 'dateTime' in event['start']
        ]

        if not filtered_events:
            print('No events found in the next 7 days.')
            return

        # Export events to CSV file
        with open('1_calendar_items.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['eventName', 'startDate', 'startTime', 'endDate', 'endTime'])

            for event in filtered_events:
                summary = event.get('summary', 'No Title')
                start = event['start']['dateTime']
                end = event['end']['dateTime']

                # Parse start and end times
                start_datetime = datetime.datetime.fromisoformat(start.replace('Z', '+00:00'))
                end_datetime = datetime.datetime.fromisoformat(end.replace('Z', '+00:00'))

                # Format the data
                start_date = start_datetime.strftime('%Y-%m-%d')
                start_time = start_datetime.strftime('%H:%M:%S')
                end_date = end_datetime.strftime('%Y-%m-%d')
                end_time = end_datetime.strftime('%H:%M:%S')

                # Write to CSV
                csvwriter.writerow([summary, start_date, start_time, end_date, end_time])

        print("Events successfully fetched.")

    except HttpError as error:
        print(f"An error occurred: {error}")

if __name__ == "__main__":
    main()
