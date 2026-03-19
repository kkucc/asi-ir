import serial
import time
from typing import Optional

COM_PORT = "COM4"
BAUD_RATE = 9600
UNITS_MM_TO_DEVICE = 10000
MOVE_DISTANCE_X_MM = 0.003
DWELL_TIME_S = 0.5      
NUM_MOVES = 1
ser: Optional[serial.Serial] = None