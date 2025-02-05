import sys
import os

# Add the parent directory (6G_RoboFUSE) to the system path
sys.path.append(os.path.abspath(os.path.join(os.getcwd(), '..')))

# Test import
try:
    # Now you can import from Sensing and mss
    from Sensing.py_mmw_main_robomaster import main as radar_main  # Radar handler from Sensing folder
    print("Import successful")
except ModuleNotFoundError as e:
    print(f"Import failed: {e}")