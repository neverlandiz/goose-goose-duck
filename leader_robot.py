import RPi.GPIO as GPIO
import time
import numpy as np
import cv2
from picamera2 import Picamera2
import socket
import threading

# Pins
GPIO.setmode(GPIO.BCM)
QUIT_BUTTON_PIN = 27
START_SEEK_BUTTON_PIN = 17
MOVE_SEQUENCE_BUTTON_PIN = 22
TEMP_QUIT_PIN =  23
LEFT_MOTOR_PWM = 20
LEFT_MOTOR_IN1 = 5
LEFT_MOTOR_IN2 = 6
RIGHT_MOTOR_PWM = 21
RIGHT_MOTOR_IN1 = 4
RIGHT_MOTOR_IN2 = 12

# Motor Pin Setup
GPIO.setup(LEFT_MOTOR_PWM, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_IN1, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_IN2, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_PWM, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)

# Physical Quit Button Pin Setup
GPIO.setup(QUIT_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(START_SEEK_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MOVE_SEQUENCE_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(TEMP_QUIT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Color Range
# Currently, red?
LOWER = np.array([80, 150, 80])
UPPER = np.array([120, 255, 255])
# 100, 200, 255

# Area
STOP = 300
# STOP = 10000
# want around 164,649?
DETECT = 500

# Boundaries
HEIGHT = 480
WIDTH = 640
CENTER_LEFT = 100
CENTER_RIGHT = 540

# Global Flags
robot_running = True
seek_running = False
move_sequence_running = False

# Motor Variables
frequency = 50
pulse_width = 0.2
pleft = GPIO.PWM(LEFT_MOTOR_PWM, frequency)
pright = GPIO.PWM(RIGHT_MOTOR_PWM, frequency)

# WiFi Setup
UDP_IP = ""
UDP_PORT = 5005
FOLLOWER_IP = "10.49.245.165"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.1)

# Button Callback
def physical_quit_callback(channel):
	global seek_running
	global robot_running
	global move_sequence_running
	print("QUIT")
	if seek_running or move_sequence_running:
		stop()
	seek_running = False
	robot_running = False
	move_sequence_running = False

def temp_quit_callback(channel):
	global seek_running
	global move_sequence_running
	print("TEMP QUIT")
	if seek_running or move_sequence_running:
		stop()
	seek_running = False
	move_sequence_running = False

def seek_callback(channel):
	global seek_running
	if not seek_running:
		seek_running = True
		seek_t = threading.Thread(target=seek_thread, daemon=True)
		seek_t.start()

def move_sequence(channel):
	global move_sequence_running
	if not move_sequence_running:
		move_sequence_running = True
		move_t = threading.Thread(target=move_sequence_thread, daemon=True)
		move_t.start()

GPIO.add_event_detect(QUIT_BUTTON_PIN, GPIO.FALLING, callback=physical_quit_callback, bouncetime=200)
GPIO.add_event_detect(START_SEEK_BUTTON_PIN, GPIO.FALLING, callback=seek_callback, bouncetime=200)
GPIO.add_event_detect(MOVE_SEQUENCE_BUTTON_PIN, GPIO.FALLING, callback=move_sequence, bouncetime=200)
GPIO.add_event_detect(TEMP_QUIT_PIN, GPIO.FALLING, callback=temp_quit_callback, bouncetime=200)

# Motor Functions
def forward(dc):
	print("Move Forward")
	
	global pulse_width
	GPIO.output(LEFT_MOTOR_IN1, GPIO.LOW)
	GPIO.output(LEFT_MOTOR_IN2, GPIO.HIGH)
	GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
	GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
	pleft.ChangeDutyCycle(dc * 1.2)
	pright.ChangeDutyCycle(dc)
	time.sleep(pulse_width)

def turn_left(dc):
	print("Turn Left")
	
	global pulse_width
	GPIO.output(LEFT_MOTOR_IN1, GPIO.HIGH)
	GPIO.output(LEFT_MOTOR_IN2, GPIO.LOW)
	GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
	GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
	pleft.ChangeDutyCycle(dc)
	pright.ChangeDutyCycle(0)
	time.sleep(pulse_width)
	
def turn_right(dc):
	print("Turn Right")
	
	global pulse_width
	GPIO.output(LEFT_MOTOR_IN1, GPIO.LOW)
	GPIO.output(LEFT_MOTOR_IN2, GPIO.LOW)
	GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
	GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
	pleft.ChangeDutyCycle(0)
	pright.ChangeDutyCycle(dc)
	time.sleep(pulse_width)

def dance(dc):
	print("Dance")

	GPIO.output(LEFT_MOTOR_IN1, GPIO.LOW)
	GPIO.output(LEFT_MOTOR_IN2, GPIO.LOW)
	GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
	GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
	pleft.ChangeDutyCycle(0)
	pright.ChangeDutyCycle(dc)
	time.sleep(5)

def backward(dc):
	print("Move Backward")
	
	global pulse_width
	GPIO.output(LEFT_MOTOR_IN1, GPIO.HIGH)
	GPIO.output(LEFT_MOTOR_IN2, GPIO.LOW)
	GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
	GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
	pleft.ChangeDutyCycle(dc)
	pright.ChangeDutyCycle(dc)
	time.sleep(pulse_width)
	
def stop():
	print("Stop")

	global pulse_width
	GPIO.output(LEFT_MOTOR_IN1, GPIO.LOW)
	GPIO.output(LEFT_MOTOR_IN2, GPIO.LOW)
	GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
	GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
	pleft.ChangeDutyCycle(0)
	pright.ChangeDutyCycle(0)
	time.sleep(pulse_width)

# Calculations
def move(object_location):
	if object_location < CENTER_LEFT:
		# Need to turn left
		turn_left(50)
	elif object_location > CENTER_RIGHT:
		# Need to turn right
		turn_right(50)
	else:
		# In the middle
		forward(50)

# Threads
def move_sequence_thread():
	global seek_running
	global move_sequence_running
	global robot_running
	
	i = 0
	while move_sequence_running and robot_running:
		if not robot_running:
			# robot quit
			move_sequence_running = False
			break
		elif seek_running:
			# mom starts seeking
			move_sequence_running = False
			break
		else:
			if i % 2 == 0:
				forward(50)
				time.sleep(5)
			else:
				turn_right(50)
				time.sleep(2)
			
			i += 1

def seek_thread():
	global robot_running
	global seek_running
	while seek_running:
		frame = camera.capture_array()
		# cv2.imshow("Frame", frame)
	
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		#print("HSV", hsv[240, 320])
		mask = cv2.inRange(hsv, LOWER, UPPER)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
	
		contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	
		if len(contours) > 0:
			# Found
			c = max(contours, key=cv2.contourArea)
			area = cv2.contourArea(c)
		
			if area > DETECT:
				x, y, w, h = cv2.boundingRect(c)
			
				# for debugging
				cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
				# cv2.imshow("Frame", frame)

				# print("AREA: ", area)
				print("Height: ", h)
			
				# detecting baby duck
				# if AREA > STOP:
				if h > STOP:
					# Arrived
					print("FOUND")
					stop()
					seek_running = False
					send_command("FOUND")
					dance(50)
					# quit
					robot_running = False
					break
				else:
					# Seek
					center = x + w // 2
					move(center)
					time.sleep(0.01)
		else:
			# Not Found, turn?
			turn_left(40)
			
		cv2.waitKey(1)

# Send and Receive
def send_command(message):
	sock.sendto(message.encode(), (FOLLOWER_IP, UDP_PORT))
	print("Send: ", message)

def receive_command():
	try:
		data, addr = sock.recvfrom(1024)
		print("Received: ", data.decode())
		return data.decode()
	except socket.timeout:
		return None

# Start Camera
camera = Picamera2()
camera.start()

# Start Motors
pleft.start(0)
pright.start(0)

# Main Loop
while robot_running:
	# communication from baby duck
	received_data = receive_command()
	if received_data == "LOST":
		if not seek_running:
			seek_running = True
			seek_t = threading.Thread(target=seek_thread, daemon=True)
			seek_t.start()
	
	time.sleep(1)

pleft.stop()
pright.stop()
camera.stop()
cv2.destroyAllWindows()
sock.close()
GPIO.cleanup()

	

