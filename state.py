from enum import Enum
import time 


class command(Enum):
    ENABLING = "ON"
    SHUTDOWN = "OFF"
    PAUSE = "PAUSE"
    EMERGENCY = "EMERGENCY"
    MOVETOSTART =  "WAIT"
    GETTINGSTARTED = "START"
    EXIT = "EXIT"
class Command(): 

    QApplication = None 
    RobotStates = None 
    AxisStates = None 
