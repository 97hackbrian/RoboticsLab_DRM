import subprocess
import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
python_script_path = os.path.join(script_dir)
sys.path.append(python_script_path)

script_dir=  python_script_path+"/robofut_tracker.py"
video_path= python_script_path+"/fulbito.mp4"

print("\n")
print(python_script_path)
subprocess.run(["python3", script_dir, video_path, "--captures", "1", "--threshold", "23.00"])
