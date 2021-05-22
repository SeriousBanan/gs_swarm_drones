#! /bin/bash

# Disable hotspot and enable NetworkManager.

sudo systemctl disable config-wlan.service
sudo systemctl stop isc-dhcp-server.service
sudo systemctl disable isc-dhcp-server.service
sudo systemctl stop hostapd.service
sudo systemctl disable hostapd.service
sudo systemctl enable NetworkManager
sudo systemctl start NetworkManager
