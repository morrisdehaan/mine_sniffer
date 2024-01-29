"""
Main rover code.
"""

import subprocess
import socket
import time
import json
from sensor.sensors import Sensors
import interface
from sensor.metal import MetalDetector
import base

PACKAGE_HEADER_SIZE = 4

if __name__ == "__main__":
    MAX_PACKAGE_BYTES = 1024 # TODO: pick appropiate number
    RTK_SOLUTION_SCRIPT = "nav/rover.sh"

    PORT = 5000
    BASE_IP = socket.gethostname() # TODO:

    manual_control = False

    # connect to base server
    client = socket.socket()
    client.connect((BASE_IP, PORT))

    sensors = Sensors(
        metal=MetalDetector()
    )

    # run RTK solution in another thread
    # subprocess.Popen(RTK_SOLUTION_SCRIPT) # TODO: uncomment

    client.setblocking(False)
    while True:
        try:
            # TODO: send header
            bytes = client.recv(MAX_PACKAGE_BYTES)

            if bytes == base.MANUAL_CONTROL_MSG:
                manual_control = True
            elif bytes == base.AUTO_CONTROL_MSG:
                manual_control = False
            elif manual_control:
                pass
        except:
            pass

        # measure sensor input
        metal = sensors.metal.detect()

        # detect mines
        # ..

        # predict mines
        # ..

        # control motors
        # ..

        # send data to base
        data = {
            "metal": metal
        }
        bytes = json.dumps(data).encode("utf-8")
        header = len(bytes).to_bytes(PACKAGE_HEADER_SIZE, "little")
        client.send(header + bytes)

        time.sleep(0.01)
    
    client.close()