import time
import sensor.shared
import RPi.GPIO as GPIO

# cm/s
SOUND_SPEED = 34300

# first 5 left side sonars, then right side
TRIG_PINS = [20, 26, 19, 6, 7,    8, 14, 18, 27, 23]
ECHO_PINS = [21, 16, 13, 12, 1,   4, 15, 17, 22, 24]

def init():
    for pin in TRIG_PINS:
        GPIO.setup(pin, GPIO.OUT)
    for pin in ECHO_PINS:
        GPIO.setup(pin, GPIO.IN)

# TODO: might want to run async as this might take a while
def measure():
    dists = []
    for trig, echo in zip(TRIG_PINS, ECHO_PINS):
        # trigger sonar
        GPIO.output(trig, True)
        time.sleep(0.000001)
        GPIO.output(trig, False)

        # measure response time
        start = time.time()
        no_response = False
        while GPIO.input(echo) == 0:
            now = time.time()
            if now - start > 0.2:
                # cancel if it takes too long
                no_response = True
                break

        if not no_response:
            start = time.time()
            while GPIO.input(echo) == 1:
                # TODO: terminate if it takes too long
                pass
            end = time.time()

            # compute distance, divide by 2 as signal moves to and fro
            dist = (end - start) * SOUND_SPEED / 2
        else:
            dist = float('inf')

        dists.append(dist)
