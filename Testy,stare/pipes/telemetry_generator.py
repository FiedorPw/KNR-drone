import sysv_ipc
import json

# Create or attach to a message queue
key = 128  # Message queue key
mq = sysv_ipc.MessageQueue(key, sysv_ipc.IPC_CREAT)

# JSON data to send
data = {
    'message': 'Hello from sender',
    'value': 42
}

# Convert JSON object to string and send it
json_data = json.dumps(data)
mq.send(json_data.encode('utf-8'))  # Encode string to bytes
print(f"Sent: {json_data}")
