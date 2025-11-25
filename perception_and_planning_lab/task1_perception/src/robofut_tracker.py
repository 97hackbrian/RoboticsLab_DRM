import cv2
import numpy as np
import argparse
import sys
import os

# Add python_scripts to path relative to this file
# We need to go up from src -> task1_perception -> perception_and_planning_lab
script_dir = os.path.dirname(os.path.abspath(__file__))
python_scripts_path = os.path.join(script_dir, '../../python_scripts')
sys.path.append(python_scripts_path)

import ball_tracker

if __name__ == '__main__':
    kalaman=ball_tracker.KalmanTracker()