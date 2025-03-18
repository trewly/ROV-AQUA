import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))) 

from Mission_planner.status import pc_status

def read_pitch():
    pitch = pc_status.read_status("pitch")
    return pitch

def read_heading():
    heading = pc_status.read_status("heading")
    return heading

def read_roll():
    roll = pc_status.read_status("roll")
    return roll