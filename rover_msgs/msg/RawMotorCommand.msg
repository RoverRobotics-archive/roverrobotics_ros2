# Motor Command sent to the Rover firmware, according to the Rover protocol v1
# Sent in conjunction with a RawData in order to form a complete request
# all items here are integers from 0-250, representing the direction and voltage applied to the motor
# 0 = full power reverse, 125 = stop motor, 250 = full power forward

uint8 left    # left motor
uint8 right   # right motor
uint8 flipper # flipper motor (might not be installed in some rovers)
