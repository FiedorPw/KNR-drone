import sysv_ipc
import json

# Attach to the message queue
key = 128  # Message queue key
mq = sysv_ipc.MessageQueue(key)

# Receive the message
message, _ = mq.receive()
json_data = message.decode('utf-8')  # Decode bytes to string
data = json.loads(json_data)  # Parse JSON string to Python object
print(f"Received: {data}")

# Clean up the message queue (optional)
mq.remove()
