""" Shared code because some modules can only be imported from the raspberry os. """

# must be divisible by 2
ROVER_MSG_HEADER_SIZE = 4

# TODO: undefine in base
# size of header of each message in bytes, where the header
#  simply gives the size of the message body
MSG_HEADER_SIZE = 2

# notifies the rover that control is set to manual
MANUAL_CONTROL_MSG = b"manual"
# notifies the rover that control is set to automatic
AUTO_CONTROL_MSG = b"auto"