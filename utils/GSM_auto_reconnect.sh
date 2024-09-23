#!/bin/bash
cd /home/KNR/KNR-dron/utils
while true; do
	if ping -c 1 1.1.1.1; then
		echo "Con up!"
	else
		./GSM_auto_conf.sh
		systemctl restart cloudflared
	fi
	sleep 1
done
