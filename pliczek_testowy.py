import json
import os
import time

log_dir="/home/KNR/KNR-dron/LOGS/Ball_status"

# Creates a new log filename with the current timestamp
def create_status_filename(self):
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return os.path.join(log_dir, f"ball_status_{current_time}.json")


def save_to_json(self):
    temp_filename = log_filename + '.temp'
    with file_lock:
        with open(temp_filename, 'a') as temp_file:
            json.dump(status_data, temp_file, indent=4)
        shutil.move(temp_filename, log_filename)
    save_count += 1
    if save_count >= 100000:
        save_count = 0
        log_filename = log_dir
        status_data = self.reset_status_data()
