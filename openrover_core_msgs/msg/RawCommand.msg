# Command sent to the Rover firmware, according to the OpenRover protocol v1
# Sent in conjunction with a RawMotorCommand in order to form a complete request


uint8 verb # Action that the rover is to perform in response to this request
uint8 arg
