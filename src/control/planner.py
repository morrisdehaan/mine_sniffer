"""
Simple random coverage algorithm.
"""

from enum import Enum
import random
import time

ROVER_RPM = 10 # TODO

# in seconds
MAX_DRIVE_TIME = 15
# time to spin 360°
MAX_SPIN_TIME = 5 # TODO: measure

# in cm
MIN_DIST = 50

class State(Enum):
    # drive forward
    Drive = 0
    # spin until obstacle is out of sight
    PrepareSpin1 = 1
    # spin until random angle is reached or obstacle is in the way
    Spin1 = 2
    # again, spin until obstacle is out of sight
    PrepareSpin2 = 3
    # spin to random angle between the two times obstacles were detected
    Spin2 = 4
    # enough spinning around, just take the first free direction
    GreedySpin = 5

class Rotation(Enum):
    Clockwise = 0
    Counterclockwise = 1

class Planner:
    def __init__(self):
        self._state = State.RandomSpin
        if random.randint(0, 1) == 0:
            self._spin_dir = Rotation.Clockwise
        else:
            self._spin_dir = Rotation.Counterclockwise
        
    def update(self, sonar: list[float]):
        blocked = detect_obstacle(sonar)

        t = time.time()

        if self._state == State.Drive:
            if blocked or t > self._drive_time:
                self._state = State.PrepareSpin1

                if random.randint(0, 1) == 0:
                    self._spin_dir = Rotation.Clockwise
                else:
                    self._spin_dir = Rotation.Counterclockwise

        if self._state == State.PrepareSpin1:
            if not blocked:
                self._state = State.Spin1

                # pick random spin time
                self._spin_time = t + random.uniform(0, MAX_SPIN_TIME)
                self._spin_start_time = t

        if self._state == State.Spin1:
            if blocked:
                self._state = State.PrepareSpin2
                self._flip_spin()
            elif t > self._spin_time:
                self._state = State.Drive
                self._drive_time = t + random.uniform(0, MAX_DRIVE_TIME)

        if self._state == State.PrepareSpin2:
            if not blocked:
                self._state = State.Spin2

                self._spin_time = t + random.uniform(0, t - self._spin_start_time)

        if self._state == State.Spin2:
            if blocked:
                self._state = State.GreedySpin
                self._flip_spin()
            elif t > self._spin_time:
                self._state = State.Drive
                self._drive_time = t + random.uniform(0, MAX_DRIVE_TIME)

        if self._state == State.GreedySpin:
            if not blocked:
                self._state = State.Drive
                self._drive_time = t + random.uniform(0, MAX_DRIVE_TIME)

    def _flip_spin(self):
        if self._spin_dir == Rotation.Clockwise:
            self._spin_dir = Rotation.Counterclockwise
        else:
            self._spin_dir = Rotation.Clockwise
    
    # TODO: weg?
    def _init_state(self):
        match self._state:
            case State.Drive:
                # at which time a new direction is picked
                self._drive_time = time.time() + random.uniform(0, MAX_DRIVE_TIME)
            case State.PrepareSpin1:
                # at which time the rover starts driving forwards
                self._spin_time = time.time() + random.uniform(0, MAX_SPIN_TIME)

def detect_obstacle(sonar: list[float]) -> bool:
    any(x <= MIN_DIST for x in sonar)