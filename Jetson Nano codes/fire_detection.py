
import jetson.inference
import jetson.utils
import argparse
import sys
import time
import serial
import Jetson.GPIO as GPIO

# LEDs Init
led_pin = [11, 13]
GPIO.setmode(GPIO.BOARD)
GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.HIGH)
GPIO.output(led_pin, GPIO.LOW)

# UART Initialization
serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

# Wait a second to let the port initialize
time.sleep(1)

model_path = "/home/pawa/jetson-inference/python/training/detection/ssd/models/fire-m1/ssd-mobilenet.onnx"
labels_path = "/home/pawa/jetson-inference/python/training/detection/ssd/models/fire-m1/labels.txt"

# parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.detectNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default=model_path, help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.2, help="minimum detection threshold to use") 

is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# load the object detection network
net = jetson.inference.detectNet(opt.network, sys.argv, opt.threshold)

# create video sources & outputs
input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
#output = jetson.utils.videoOutput("rtp://10.0.0.3:5555")
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv+is_headless)

# process frames until the user exits
GPIO.output(led_pin[0], GPIO.HIGH) 
while True:
	
	img = input.Capture()
	detections = net.Detect(img, overlay=opt.overlay)
	GPIO.output(led_pin[1], GPIO.LOW) 
	if len(detections) > 0:
		print("detected {:d} objects in image".format(len(detections)))
		
		for detection in detections:
			GPIO.output(led_pin[1], GPIO.HIGH)  # RED Led on
			center = int(detection.Center[0])   # X coord. of object
			print(f"{center:04d}")
			serial_port.write((f"{center:04d}\r\n").encode()) # Send to UART
	else:
		print("xxxx")
		serial_port.write("xxxx\r\n".encode())  # Send to UART

	output.Render(img)
	output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

	# print out performance info
	#net.PrintProfilerTimes()

	# exit on input/output EOS
	if not input.IsStreaming() or not output.IsStreaming():
		GPIO.output(led_pin[0], GPIO.LOW) 
		GPIO.output(led_pin[1], GPIO.LOW) 
		break


