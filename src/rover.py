"""
Main rover code.
"""

# TODO: mark raspberry and base station os code

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
import sensor.sonar as sonar
from sensor.ircam import IRCam
import shared

METAL_ARDUINO_PORT = "/dev/ttyUSB0" # "COM4"
METAL_ARDUINO_BAUD = 9600
MOTOR_ARDUINO_PORT = "/dev/ttyACM0" # "COM5"
MOTOR_ARDUINO_BAUD = 115200

""" Parsed message into separate commands. """
def parse_msg(msg: bytes) -> list[bytes]:
    cmds = []
    i = 0
    while i < len(msg):
        size = int.from_bytes(msg[i:i+base.MSG_HEADER_SIZE], "little")
        cmds.append(msg[i+base.MSG_HEADER_SIZE:i+base.MSG_HEADER_SIZE+size])
        i += base.MSG_HEADER_SIZE + size
    return cmds

if __name__ == "__main__":
    MAX_PACKAGE_BYTES = 1024 # TODO: pick appropiate number
    RTK_SOLUTION_SCRIPT = "nav/rover.sh"

    PORT = 5000
    # change to the IPv4 address of the base 
    BASE_IP = "192.168.55.241"

    control = controller.Controller(MOTOR_ARDUINO_PORT, MOTOR_ARDUINO_BAUD)
    manual_control = False

    # connect to base server
    client = socket.socket()
    client.connect((BASE_IP, PORT))

    sensors = Sensors(
        metal=MetalDetector(METAL_ARDUINO_PORT, METAL_ARDUINO_BAUD),
        ircam=IRCam()
    )

    # run RTK solution in another thread
    # subprocess.Popen(RTK_SOLUTION_SCRIPT) # TODO: uncomment

    #client.setblocking(False) # TODO: cannot use this one?
    client.settimeout(0.1)
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
        except socket.timeout:
            pass

        # measure sensor input
        metal = sensors.metal.detect()
        sonar_dists = sonar.measure()
        ircam_updated = sensors.ircam.update()

        # detect mines
        # ..

        # predict mines
        # ..

        # control motors
        # ..

        # send data to base
        data = {
            "metal": metal,
            "sonar": sonar_dists
        }

        if ircam_updated:
            # ir-camera data is serialized separately, because json can only handle
            #  python floats which are way too big
            ircam_bytes = sensors.ircam.img.tobytes()
        else:
            ircam_bytes = b""
        
        bytes = json.dumps(data).encode("utf-8")
        header = len(bytes).to_bytes(int(shared.ROVER_MSG_HEADER_SIZE / 2), "little")\
               + len(ircam_bytes).to_bytes(int(shared.ROVER_MSG_HEADER_SIZE / 2), "little")
               
        # we send the header, sensor data (minus ir-camera) and ir-camera data
        client.send(header + bytes + ircam_bytes)

        time.sleep(0.01) # TODO: weg?

        # TODO: kill switch?
    
    client.close()
