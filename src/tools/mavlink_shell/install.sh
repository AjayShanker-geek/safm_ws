#!/bin/bash

# Install MAVLink Shell dependencies
echo "Installing MAVLink Shell Deps..."
sudo apt update
sudo apt install -y python3-pip
pip3 install --user pymavlink
