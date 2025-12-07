import RPi.GPIO as GPIO
import time
import socket

TRIG_A = 25
ECHO_A = 26

TRIG_B = 6
ECHO_B = 5

IN1_A = 17
IN2_A = 27
IN1_B = 23
IN2_B = 24
PWM_A = 13
PWM_B = 19
BUZZER = 16  # buzzer on BCM16, short leg to GND

UDP_IP = ""
UDP_PORT = 5005
LEADER_IP = "10.49.243.186"

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_A, GPIO.OUT)
GPIO.setup(ECHO_A, GPIO.IN)

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_B, GPIO.OUT)
GPIO.setup(ECHO_B, GPIO.IN)

GPIO.setup([IN1_A, IN2_A, IN1_B, IN2_B, PWM_A, PWM_B], GPIO.OUT)
GPIO.setup(BUZZER, GPIO.OUT)

pwm_a = GPIO.PWM(PWM_A, 1000)
pwm_b = GPIO.PWM(PWM_B, 1000)
pwm_a.start(0)
pwm_b.start(0)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.1)

lost_flag = False
stop_flag = True
last_move_time = time.time()  # last time the robot was moving

GPIO.output(TRIG_A, False)
GPIO.output(TRIG_B, False)
GPIO.output(BUZZER, False)
print("waiting")
time.sleep(2)


def get_distance(TRIG, ECHO):
    GPIO.output(TRIG, 1)
    time.sleep(0.00001)
    GPIO.output(TRIG, 0)

    timeout = time.time() + 0.02
    while GPIO.input(ECHO) == 0:
        if time.time() > timeout:
            return None
    start = time.time()

    timeout = time.time() + 0.02
    while GPIO.input(ECHO) == 1:
        if time.time() > timeout:
            return None
    end = time.time()

    duration = end - start
    distance = (duration * 34300) / 2
    return distance


def get_two_distances():
    d_b = get_distance(TRIG_B, ECHO_B)
    time.sleep(0.02)
    d_a = get_distance(TRIG_A, ECHO_A)

    return d_a, d_b


def distance_to_speed(dist_cm):
    global lost_flag
    if dist_cm is None or dist_cm <= 7:
        return 0
    elif dist_cm >= 80:
        # lost due to far distance
        lost_flag = True
        GPIO.output(BUZZER, True)
        time.sleep(1)
        GPIO.output(BUZZER, False)
        return 0
    else:
        return (min(50, dist_cm * 20 / 100) + 50)


def set_motor_speeds(speed_a, speed_b):

    spd_a = max(0, min(100.0, float(speed_a)))
    spd_b = max(0, min(100.0, float(speed_b)))

    if spd_a == 0 and spd_b == 0:

        GPIO.output(IN1_A, False)
        GPIO.output(IN2_A, False)
        GPIO.output(IN1_B, False)
        GPIO.output(IN2_B, False)
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        return

    GPIO.output(IN1_A, 0)
    GPIO.output(IN2_A, 1)
    GPIO.output(IN1_B, 1)
    GPIO.output(IN2_B, 0)
    pwm_a.ChangeDutyCycle(spd_a)
    pwm_b.ChangeDutyCycle(spd_b)

# NEW: params for alignment
ALIGN_THRESHOLD = 2.0    # cm difference to start turning
ALIGN_BOOST     = 20.0   # extra speed for the farther side

try:
    while stop_flag:
        dist_a, dist_b = get_two_distances()
        if dist_a is not None and dist_b is not None:
            front_dist = min(dist_a, dist_b)

        elif dist_a is not None:
            front_dist = dist_a

        elif dist_b is not None:
            front_dist = dist_b
        else:
            front_dist = None

        base_speed = distance_to_speed(front_dist)
        speed_a = base_speed
        speed_b = base_speed

        if dist_a is not None and dist_b is not None:
            diff = dist_a - dist_b

            if diff > ALIGN_THRESHOLD:
                speed_a = min(base_speed + ALIGN_BOOST, 100)
                speed_b = base_speed

            elif diff < -ALIGN_THRESHOLD:
                speed_b = min(base_speed + ALIGN_BOOST, 100)
                speed_a = base_speed

        set_motor_speeds(speed_a, speed_b)

        # lost if stopped for 5 seconds
        if speed_a == 0 and speed_b == 0:
            if time.time() - last_move_time >= 5.0:
                lost_flag = True
                GPIO.output(BUZZER, True)
                time.sleep(1)
                GPIO.output(BUZZER, False)
        else:
            last_move_time = time.time()

        # notify leader robot if lost
        if lost_flag:
            send_command("LOST")
            while True:
                received_message = receive_command()
                if received_message == "FOUND":
                    lost_flag = False
                    GPIO.output(BUZZER, False)
                    # dance
                    set_motor_speeds(50, -50)
                    time.sleep(5)
                    stop_flag = False
                    break

        print(
            f"A:{fmt_dist(dist_a)} cm | "
            f"B:{fmt_dist(dist_b)} cm | "
            f"Front: {fmt_dist(front_dist)} cm | "
            f"SpeedA:{speed_a:.0f}% SpeedB :{speed_b:.0f}%"
        )
        
        time.sleep(0.2)

except KeyboardInterrupt:
    print("stop")

pwm_a.stop()
pwm_b.stop()
GPIO.cleanup()
sock.close()
    
