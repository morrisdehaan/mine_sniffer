""" Shared code because some sensor modules can only be imported from the raspberry os. """

from typing import NamedTuple

SONAR_COUNT = 10
# number of sonars on the left or right side
SONAR_BLOCK_COUNT = 5
# maximum distance measured by sonars in meters
MAX_SONAR_DIST = 10.0
# angle that the sonars on the left or right side cover
SONAR_BLOCK_ANGLE = 2.09 # TODO: measure actual angle

class Coord(NamedTuple):
    lat: float
    long: float