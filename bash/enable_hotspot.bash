#! /bin/bash

# Disable hotspot and enable NetworkManager.

sudo systemctl stop NetworkManager
sudo systemctl disable NetworkManager
sudo systemctl enable config-wlan
sudo systemctl start config-wlan
sudo systemctl enable isc-dhcp-server
sudo systemctl start isc-dhcp-server
sudo systemctl enable hostapd
sudo systemctl start hostapd
