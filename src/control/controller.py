"""
A basic controller that can send some commands to the motor driver arduino.
"""

import serial

VALID_CMDS = ["w", "a", "s", "d", "up", "down", "left", "right", " "]

class IllegalControlCmd(Exception):
    pass

class Controller:
    def __init__(self, serial_port: str, baudrate: int):
        self.serial = serial.Serial(serial_port, baudrate)

    def send(self, cmd: str):
        if not is_cmd_valid(cmd):
            raise IllegalControlCmd(f"'{cmd}' is not a valid command!")
        
        self.serial.write(cmd)

""" Returns `True` if the command is valid. """
def is_cmd_valid(cmd: str) -> bool:
    return cmd in VALID_CMDS