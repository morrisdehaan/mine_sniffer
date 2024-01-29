"""
Main base code.
"""

import socket
import interface
import time
import keyboard as kb

# TODO: fix import errors

# notifies the rover that control is set to manual
MANUAL_CONTROL_MSG = b"manual"
# notifies the rover that control is set to automatic
AUTO_CONTROL_MSG = b"auto"

if __name__ == "__main__":
    PORT = 5000
    MAX_PACKAGE_BYTES = 4096 # TODO: pick appropiate number

    # TODO: run navigation bash
    
    # initialize interface
    itf = interface.Interface()

    # initialize server
    print(socket.gethostname())
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((socket.gethostname(), PORT))

    server.listen(1)

    print("Waiting for rover to connect...")
    rover_conn, rover_addr = server.accept() # TODO: make quitable if this does not happen..
    print(f"Rover connected from {rover_addr}!")

    server.setblocking(False)
    while True:
        try:
            bytes = rover_conn.recv(MAX_PACKAGE_BYTES)
            update = interface.parse_packet(bytes)
        except socket.timeout:
            update = None

        control_switch = itf.update(update)

        if control_switch:
            if itf.manual_control_requested():
                rover_conn.send(MANUAL_CONTROL_MSG)
            else:
                rover_conn.send(AUTO_CONTROL_MSG)

        if itf.manual_control_requested():
            # send user control 
            # TODO: bit hacky still
            if kb.is_pressed("w"):
                rover_conn.send("w")
            elif kb.is_pressed("a"):
                rover_conn.send("a")
            elif kb.is_pressed("s"):
                rover_conn.send("s")
            elif kb.is_pressed("d"):
                rover_conn.send("d")
            elif kb.is_pressed("q"):
                rover_conn.send("q")
            elif kb.is_pressed("e"):
                rover_conn.send("e")

    rover_connc.close()
