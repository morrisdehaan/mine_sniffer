"""
Main base code.
"""

import socket
import interface
import keyboard as kb
import control.controller as controller
import time

# TODO: rename package to msg everywhere
# TODO: fix import errors

# size of header of each message in bytes, where the header
#  simply gives the size of the message body
MSG_HEADER_SIZE = 2

# notifies the rover that control is set to manual
MANUAL_CONTROL_MSG = b"manual"
# notifies the rover that control is set to automatic
AUTO_CONTROL_MSG = b"auto"

def send_msg(conn: socket.socket, msg: bytes):
    header = len(msg).to_bytes(MSG_HEADER_SIZE, byteorder="little")

    conn.send(header + msg)

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
    rover_conn, rover_addr = None, None
    update = None

    rover_connected = False
    server.setblocking(False)

    # fps management
    lastframetime = time.time()
    time_delta = 1.0 / 60.0

    while True:
        control_switch = itf.update(update)

        if not rover_connected:
            try:
                rover_conn, rover_addr = server.accept()
                rover_connected = True
                print(f"Rover connected from {rover_addr}!")
            except:
                continue                    
        else:
            try:
                bytes = rover_conn.recv(MAX_PACKAGE_BYTES)
                update = interface.parse_packet(bytes)
            except: # TODO:
                update = None

            if control_switch:
                if itf.manual_control_requested():
                    send_msg(rover_conn, MANUAL_CONTROL_MSG)
                else:
                    send_msg(rover_conn, AUTO_CONTROL_MSG)

            if itf.manual_control_requested():
                # send user control
                for cmd in controller.VALID_CMDS:
                    if kb.is_pressed(cmd):
                        send_msg(rover_conn, cmd.encode("utf-8"))
                        break

        # kill switch
        if kb.is_pressed("-"):
            break

        # manage fps
        while time.time() < lastframetime + time_delta:
            time.sleep(0.01)
        lastframetime = time.time()
    
    rover_conn.close()
    server.close()
