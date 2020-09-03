# Data packet sent from the Rover firmware, according to the OpenRover protocol v1
# sent by the rover in response to a Data Request.

uint8 which   # which data element this represents (e.g. Battery A Voltage, Firmware Version)
byte[2] value # raw binary data sent by the rover. How to decode this data depends on the 'which' field
