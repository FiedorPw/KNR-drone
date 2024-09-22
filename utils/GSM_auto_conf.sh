#!/bin/bash

# Step 1: Get modem ID
modem_id=$(mmcli -L | grep -oP '(?<=Modem/)\d+')
echo "Modem ID: $modem_id"

# Step 2: Check modem details and connect
if [ -z "$modem_id" ]; then
    echo "No modem found. Exiting."
    exit 1
fi

sudo mmcli -m "$modem_id" --simple-connect="apn=internet,ip-type=ipv4"

# Step 3: Get bearer ID
bearer_id=$(mmcli -m "$modem_id" | grep -oP '(?<=Bearer\s+\|\s+paths:\s+/org/freedesktop/ModemManager1/Bearer/)\d+')
echo "Bearer ID: $bearer_id"

if [ -z "$bearer_id" ]; then
    # Create a new bearer if none exists
    echo "No bearer found. Creating a new bearer..."
    sudo mmcli -m "$modem_id" --create-bearer="apn=internet"
    bearer_id=$(mmcli -m "$modem_id" | grep -oP '(?<=Bearer\s+\|\s+paths:\s+/org/freedesktop/ModemManager1/Bearer/)\d+')
    echo "New Bearer ID: $bearer_id"
fi

# Step 4: Activate the bearer
sudo mmcli -m "$bearer_id"

# Step 5: Get network interface (usually wwan0)
interface=$(ip a | grep -oP 'wwan\d+')
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
