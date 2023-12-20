import time
import RPi.GPIO as GPIO

# cm/s
SOUND_SPEED = 34300

if __name__ == "__main__":
    pins = [17, 27, 22, 23, 24]

    GPIO.setmode(GPIO.BCM)
    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)

    while True:
        for pin in pins:
            # trigger sonar
            GPIO.output(pin, True)
            time.sleep(0.00001)
            GPIO.output(pin, False)

            # measure response time
            while GPIO.input(pin) == 0:
                pass

            start = time.time()
            while GPIO.input(pin) == 1:
                pass
            end = time.time()

            # compute distance, divide by 2 as signal moves to and fro
            dist = (end - start) * SOUND_SPEED / 2
            print(f"{dist}cm")

        print()
        time.sleep(0.2)