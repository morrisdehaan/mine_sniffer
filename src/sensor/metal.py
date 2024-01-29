import serial
import time
from typing import Tuple, Optional

METAL_DETECTOR_COUNT = 3

# TODO: remove
# detector difference values are stored in 32 bit int
BYTES_PER_DETECTOR = 4
BYTES_PER_BATCH = METAL_DETECTOR_COUNT * BYTES_PER_DETECTOR

SERIAL_PORT = "COM4" # TODO: fix

# separation character between samples
SAMPLE_SEP = "\n"
# separation character between detectors
DETECTOR_SEP = " "

class MetalDetector:
    def __init__(self):
        self.usb = serial.Serial(port=SERIAL_PORT, baudrate=9600, timeout=0.0)

        # used for collecting bits of sensory data
        self.data = ""

    """ Returns the reading for each metal detector if available. """
    def detect(self) -> Optional[list[Tuple[int, float]]]:
        # take last reading
        bytes = self.usb.read_all()

        if len(bytes) > 0:
            data = bytes.decode("utf-8")
            
            if data.count(SAMPLE_SEP) < 2:
                self.data += data
            else:
                self.data = data
            
            # check if a complete value for each sensor has been returned
            if self.data.count(SAMPLE_SEP) >= 2:
                now = time.time() # TODO: is this timing desired?

                idx1 = self.data.rfind(SAMPLE_SEP)
                idx0 = self.data[:idx1].rfind(SAMPLE_SEP)

                res = [(int(v), now) for v in self.data[idx0+1:idx1].split(DETECTOR_SEP)[:-1]]    
                self.data = ""
                return res    
        return None