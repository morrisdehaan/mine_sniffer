from typing import NamedTuple
from dataclasses import dataclass
from sensor.metal import MetalDetector
#from sensor.ircam import IRCam # TODO

# TODO: define elsewhere
class Coord(NamedTuple):
    lat: float
    long: float

# sensor readings
@dataclass
class SensorData:
    sonar: list[float]
    metal: list[float]
    ircam: list[float]
    coord: Coord

@dataclass
class Sensors:
    metal: MetalDetector