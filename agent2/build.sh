#!/bin/bash

# Dependencies needed to run the agent

# Create a virtual environment
python3 -m venv venv

# Activate the virtual environment
source venv/bin/activate

# Install python3-pip (if not already installed)
sudo apt-get install python3-pip

# Install numpy within the virtual environment
pip install numpy