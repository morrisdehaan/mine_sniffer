from typing import NamedTuple
from sensor.metal import MetalDetector
import sensor.sonar as sonar
from sensor.ircam import IRCam

class Sensors:
    """ Initializes all sensors (including sonar). """
    def __init__(self, metal=MetalDetector, ircam=IRCam):
        self.metal = metal
        self.ircam = ircam

        sonar.init()

