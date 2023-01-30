# ENPH353_Controller_Package

This repository is meant for holding all scripts that partake in the active and autonomous controlling of the robot during the competition.

| Source File  |Description |
| ------------- | ------------- |
| time_trials.py | Used for completing time trials, uploads timer start message, turns for a few seconds, then uploads timer end message |
| parking_spot.py  | Used to keep track of the current parking spot that the license plate is being read from  |
| license_plate.py  | Used to slice raw image feed to find licenses plate from CNN |
| pid.py  | Control method used to traverse the crosswalk and the corner afterwards  |
| crosswalk_detector.py | Used to detect crosswalk and pedestrain to determine when it is safe to cross |
| controller_main.py | Main controller of the robot, which contains imitation learning CNN and reads from publishers to change states  |

