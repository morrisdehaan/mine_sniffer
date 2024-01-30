"""
Main rover code.
"""

import subprocess
import socket
import time
import json
import sys
from sensor.sensors import Sensors
import interface
from sensor.metal import MetalDetector
import base
import control.controller as controller

PACKAGE_HEADER_SIZE = 4

METAL_ARDUINO_PORT = "COM4"
METAL_ARDUINO_BAUD = 9600
MOTOR_ARDUINO_PORT = "COM5"
MOTOR_ARDUINO_BAUD = 115200

""" Parsed message into separate commands. """
def parse_msg(msg: bytes) -> list[bytes]:
    cmds = []
    i = 0
    while i < len(msg):
        size = msg[i:i+base.MSG_HEADER_SIZE]
        cmds.append(msg[i+base.MSG_HEADER_SIZE:i+base.MSG_HEADER_SIZE+size])
        i += base.MSG_HEADER_SIZE + size
    return cmds

if __name__ == "__main__":
    MAX_PACKAGE_BYTES = 1024 # TODO: pick appropiate number
    RTK_SOLUTION_SCRIPT = "nav/rover.sh"

    PORT = 5000
    BASE_IP = socket.gethostname() # TODO:

    control = controller.Controller(MOTOR_ARDUINO_PORT, MOTOR_ARDUINO_BAUD)
    manual_control = False

    # connect to base server
    client = socket.socket()
    client.connect((BASE_IP, PORT))

    sensors = Sensors(
        metal=MetalDetector(METAL_ARDUINO_PORT)
    )

    # run RTK solution in another thread
    # subprocess.Popen(RTK_SOLUTION_SCRIPT) # TODO: uncomment

    client.setblocking(False)
    while True:
        try:
            bytes = client.recv(MAX_PACKAGE_BYTES)

            cmds = parse_msg(bytes)

            for cmd in cmds:
                if cmd == base.MANUAL_CONTROL_MSG:
                    manual_control = True
                elif cmd == base.AUTO_CONTROL_MSG:
                    manual_control = False
                # otherwise it is a manual control command
                elif manual_control:
                    try:
                        control.send(cmd.decode("utf-8"))
                    except controller.IllegalControlCmd:
                        print("Illegal control command received!", file=sys.stderr)
        except: # TODO: what error?
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

        time.sleep(0.01) # TODO: weg?

        # TODO: kill switch?
    
    client.close()