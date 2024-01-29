import time
#import RPi.GPIO as GPIO

# TODO: use code on raspberry
SONAR_COUNT = 10
# number of sonars on the left or right side
SONAR_BLOCK_COUNT = 5
# maximum distance measured by sonars in meters
MAX_SONAR_DIST = 10.0
# angle that the sonars on the left or right side cover
SONAR_BLOCK_ANGLE = 2.09 # TODO: measure actual angle

class Sonar:
    def __init__(self):
        self.__trig_pins = []
        
        # distances in centimeters
        self.dists = []
        # TODO

    def update(self):
        pass

# cm/s
SOUND_SPEED = 34300


# if __name__ == "__main__":
#     pins = [25, 27, 22, 23, 24]

#     GPIO.setmode(GPIO.BCM)
#     for pin in pins:
#         GPIO.setup(pin, GPIO.OUT)

#     while True:
#         for pin in pins:
#             # trigger sonar
#             GPIO.output(pin, True)
#             time.sleep(0.00001)
#             GPIO.output(pin, False)

#             # measure response time
#             while GPIO.input(pin) == 0:
#                 pass

#             start = time.time()
#             while GPIO.input(pin) == 1:
#                 pass
#             end = time.time()

#             # compute distance, divide by 2 as signal moves to and fro
#             dist = (end - start) * SOUND_SPEED / 2
#             print(f"{dist}cm")

#         print()
#         time.sleep(0.2)