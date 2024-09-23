#!/bin/bash

# Step 0: Restart ModemManager service
echo "Restarting ModemManager service..."
sudo systemctl restart ModemManager

# Optional: Wait for ModemManager to restart
sleep 5

# Step 1: Wait for the modem to be detected
echo "Waiting for modem to be detected..."
while true; do
    modem_id=$(mmcli -L | grep -o 'Modem/[0-9]\+' | grep -o '[0-9]\+')
    if [ -n "$modem_id" ]; then
        echo "Modem ID: $modem_id"
        break
    else
        sleep 1
    fi
done

# Step 2: Connect the modem
echo "Connecting the modem..."
if sudo mmcli -m "$modem_id" --simple-connect="apn=internet,ip-type=ipv4"; then
    echo "Successfully connected the modem"
else
    echo "Failed to connect the modem"
    exit 1
fi

# Step 3: Get bearer ID using --output-keyvalue
echo "Getting bearer ID..."
bearer_info=$(mmcli -m "$modem_id" --output-keyvalue | grep 'bearers')

# Extract the bearer path
bearer_path=$(echo "$bearer_info" | grep -o '/org/freedesktop/ModemManager1/Bearer/[0-9]\+')
bearer_id=$(echo "$bearer_path" | grep -o '[0-9]\+')
echo "Bearer ID: $bearer_id"

if [ -z "$bearer_id" ]; then
    # Create a new bearer and capture the bearer path directly from the output
    echo "No bearer found. Creating a new bearer..."
    bearer_output=$(sudo mmcli -m "$modem_id" --create-bearer="apn=internet" --output-keyvalue)
    echo "$bearer_output"
    bearer_path=$(echo "$bearer_output" | grep -o 'Bearer=.*' | cut -d'=' -f2)
    bearer_id=$(echo "$bearer_path" | grep -o '[0-9]\+')
    echo "New Bearer ID: $bearer_id"
fi

# Step 4: Activate the bearer using modem ID (-m) and bearer path
sudo mmcli -m "$modem_id" --activate-bearer="$bearer_path"

# Step 5: Get network interface (usually wwan0)
interface=$(ip link | grep -o 'wwan[0-9]\+' | head -n 1)
echo "Network interface: $interface"

# Step 6: Bring up the network interface
if [ -z "$interface" ]; then
    echo "No network interface found. Exiting."
    exit 1
fi

sudo ip link set "$interface" up

# Step 7: Request IP address with DHCP
sudo dhclient "$interface"

echo "Modem connection established on $interface."
