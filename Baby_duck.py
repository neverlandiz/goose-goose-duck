#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# === Ultrasonic pins (BCM) ===
# Sensor A: "old" sensor, e.g. left side
TRIG_A = 16
ECHO_A = 26

# NEW: Sensor B: new ultrasonic, e.g. right side
TRIG_B = 6      
ECHO_B = 5      

# === Motor pins (BCM) ===
IN1A = 17
IN2A = 27
IN1B = 23
IN2B = 24
PWMA = 13
PWMB = 19

GPIO.setmode(GPIO.BCM)

# Ultrasonic GPIO setup
GPIO.setup(TRIG_A, GPIO.OUT)
GPIO.setup(ECHO_A, GPIO.IN)

GPIO.setup(TRIG_B, GPIO.OUT)   
GPIO.setup(ECHO_B, GPIO.IN)    

# Motor GPIO setup
GPIO.setup(IN1A, GPIO.OUT)
GPIO.setup(IN2A, GPIO.OUT)
GPIO.setup(IN1B, GPIO.OUT)
GPIO.setup(IN2B, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)

# PWM for motors
pwm_a = GPIO.PWM(PWMA, 1000)
pwm_b = GPIO.PWM(PWMB, 1000)
pwm_a.start(0)
pwm_b.start(0)

GPIO.output(TRIG_A, False)
GPIO.output(TRIG_B, False)     

print("Waiting for ultrasonic sensors to settle...")
time.sleep(2)

def fmt_dist(d): 
    if d is None:
        return "None"
    return f"{d:.1f}"


def get_distance(trig_pin, echo_pin):   # CHANGED: now takes pins as args
    """Measure distance (cm) from one ultrasonic sensor."""
    # Send 10 us pulse to TRIG
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    # Wait for ECHO to go high (start)
    timeout = time.time() + 0.02  # 20 ms timeout
    while GPIO.input(echo_pin) == 0:
        if time.time() > timeout:
            return None
    start = time.time()

    # Wait for ECHO to go low (end)
    timeout = time.time() + 0.02
    while GPIO.input(echo_pin) == 1:
        if time.time() > timeout:
            return None
    end = time.time()

    duration = end - start             # seconds
    distance = (duration * 34300) / 2  # cm
    return distance

# NEW: read both sensors with a small time gap to avoid interference
def get_two_distances():
    d_a = get_distance(TRIG_A, ECHO_A)
    time.sleep(0.03)  # small delay between sensors to reduce crosstalk
    d_b = get_distance(TRIG_B, ECHO_B)
    return d_a, d_b

# Same distance→speed mapping as before
def distance_to_speed(dist_cm):
    """0 cm -> 0%; 50 cm+ -> 100%; linear in between."""
    if dist_cm is None:
        return 0
    if dist_cm < 0 or dist_cm > 200:
        return 0
    if dist_cm <= 0:
        return 0
    if dist_cm >= 50.0:
        return 100.0
    return dist_cm / 50.0 * 100.0

# NEW: per-wheel speed control
def set_motor_speeds(speed_a, speed_b):
    """Set motor A and B forward with given speeds (0–100%)."""
    spd_a = max(0.0, min(100.0, float(speed_a)))
    spd_b = max(0.0, min(100.0, float(speed_b)))

    if spd_a == 0 and spd_b == 0:
        # Stop all
        GPIO.output(IN1A, False)
        GPIO.output(IN2A, False)
        GPIO.output(IN1B, False)
        GPIO.output(IN2B, False)
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        return

    # Forward direction (swap True/False if wiring is reversed)
    GPIO.output(IN1A, True)
    GPIO.output(IN2A, False)
    GPIO.output(IN1B, True)
    GPIO.output(IN2B, False)

    pwm_a.ChangeDutyCycle(spd_a)
    pwm_b.ChangeDutyCycle(spd_b)

# NEW: params for alignment
ALIGN_THRESHOLD = 5.0    # cm difference to start turning
ALIGN_BOOST     = 20.0   # extra speed for the farther side

try:
    while True:
        dist_a, dist_b = get_two_distances()

        # Choose front distance for basic follow logic
        if dist_a is not None and dist_b is not None:
            front_dist = min(dist_a, dist_b)
        elif dist_a is not None:
            front_dist = dist_a
        elif dist_b is not None:
            front_dist = dist_b
        else:
            front_dist = None

        base_speed = distance_to_speed(front_dist)

        # Default: go straight, both wheels same speed
        speed_a = base_speed
        speed_b = base_speed

        # If both distances valid, check lateral offset
        if dist_a is not None and dist_b is not None:
            diff = dist_a - dist_b  # >0: A side farther; <0: B side farther

            if diff > ALIGN_THRESHOLD:
                # Sensor A sees farther → target turns to B
                # make motor A faster
                speed_a = min(base_speed + ALIGN_BOOST, 100.0)
                speed_b = base_speed
            elif diff < -ALIGN_THRESHOLD:
                # Sensor B sees farther
                speed_b = min(base_speed + ALIGN_BOOST, 100.0)
                speed_a = base_speed
            # |diff| <= threshold: keep straight，speed_a / speed_b same

        set_motor_speeds(speed_a, speed_b)

        print(f"A: {fmt_dist(dist_a)} cm | "
              f"B: {fmt_dist(dist_b)} cm | "
              f"Front: {fmt_dist(front_dist)} cm | "
              f"SpeedA: {speed_a:.0f}%  SpeedB: {speed_b:.0f}%")


        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopped by user")

    try:
        pwm_a.stop()
        pwm_b.stop()
    except NameError:
        pass
    GPIO.cleanup()
