# Startup program - startup.py

import subprocess
import os

# Defining the work folder
os.chdir("/home/pawa/jetson-inference/python/examples")

# Password for root user and the command to run
pwd = "********"
cmd = "sudo python3 fire_detection.py --model='/home/pawa/jetson-inference/python/training/detection/ssd/models/DS6/ssd-mobilenet.onnx' --labels='/home/pawa/jetson-inference/python/training/detection/ssd/models/DS6/labels.txt' --input-blob=input_0 --output-cvg=scores --output-bbox=boxes --threshold=0.5 csi://0"

# Activate the subprocess from the python file
subprocess.call('echo {} | sudo -S {}'.format(pwd, cmd), shell=True)
