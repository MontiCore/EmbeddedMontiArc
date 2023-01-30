#!/usr/bin/env python

import signal
import sys
import time
import rospy
import subprocess
import numpy as np
import export_traj


from optparse import OptionParser
from rosconnector import RosConnector

TRAINING_MODE = 0
PLAY_MODE = 1
EVAL_MODE = 2


def signal_handler(sig, frame):
    print("Starting shutdown")
    sys.exit(0)

def shut_down_other_node():
    subprocess.call(["rosnode", "kill", "traj_node"])

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    parser = OptionParser(usage="usage: %prog [options]")
    parser.add_option(
        "-e", "--environment", dest="environment", help="environment string"
    )
    parser.add_option(
        "-v",
        "--verbose",
        action="store_true",
        dest="verbose",
        help="show logging information",
    )
    parser.add_option(
        "-t", "--training", action="store_true", dest="training", help="training mode"
    )
    parser.add_option(
        "-p", "--play", action="store_true", dest="play", help="play mode"
    )

    options, args = parser.parse_args()
    if not options.environment:
        parser.error("Enviroment string not given")

    mode_options = 0
    mode = TRAINING_MODE
    if options.training:
        mode = TRAINING_MODE
        mode_options += 1
    if options.play:
        mode = PLAY_MODE
        mode_options += 1
    if mode_options > 1:
        parser.error("more than one mode is not allowed")

    verbose = options.verbose
    connector = RosConnector(options.environment, verbose)
    
    if mode == PLAY_MODE:
        print("Start Play Mode")
        time.sleep(3)
        connector.reset()
        export_traj.
        while not connector.is_goalReached or connector.is_crash:
            time.sleep(1)
    else:  # training mode
        print("Start Training Mode")
        while True:
            time.sleep(0.3)
            
    
    if connector.is_goalReached:
        print("Goal is reached")
    elif connector.is_crash:
        print("TurtleBot is Crashed")
        
    connector.shutdown()
    rospy.on_shutdown(shut_down_other_node)
