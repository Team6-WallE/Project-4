import time
from datetime import datetime
import sys
import logging
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler, LoggingEventHandler
import os

folder_to_track = '/home/meena/Desktop/logImages'
new_path = '/home/meena/Desktop/test'

class Handle(FileSystemEventHandler):
    def on_modified(self, event):
        date = datetime.today().strftime('%Y-%m-%d')
        folder_destination = new_path + '/' + str(date)
        
        if not os.path.exists(folder_destination):
            os.makedirs(folder_destination)
            for filename in os.listdir(folder_to_track):
                src = folder_to_track + '/' + filename
                new_dest = folder_destination + '/' + filename
                os.rename(src,new_dest)
        else:
            for filename in os.listdir(folder_to_track):
                src = folder_to_track + '/' + filename
                new_dest = folder_destination + '/' + filename
                os.rename(src,new_dest)
        logging.log(logging.INFO, f'{event.event_type} {event.src_path}')

    
logging.basicConfig(filename='patrol_log.log',
                    filemode='w',
                    level=logging.INFO,
                    format='%(asctime)s - %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')
path = sys.argv[1] if len(sys.argv) > 1 else '.'

observer = Observer()
event_handler = Handle()
observer.schedule(event_handler,folder_to_track, recursive=True)
print("Watchdog starts")
observer.start()
print("Watchdog on")

try:
    while True:
        time.sleep(60)
        print("Folder checking")
except KeyboardInterrupt:
    observer.stop()
    print("Watchdog stopped")

observer.join()