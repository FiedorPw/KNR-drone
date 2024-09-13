import os
import json

log_dir = "/home/KNR/KNR-dron/LOGS/"

def get_last_modified_file(dir_path):
    files = [os.path.join(dir_path, f) for f in os.listdir(dir_path) if os.path.isfile(os.path.join(dir_path, f))]
    if not files:
        return None
    return max(files, key=os.path.getmtime)

# Function to read telemetry data from JSON file
def read_last_json_log():
    log_path = get_last_modified_file(log_dir)
    if log_path is None:
        raise FileNotFoundError("No log files found.")
    with open(log_path, 'r') as json_file:
        return json.load(json_file)
if __name__ == '__main__':
    print(read_last_json_log())
