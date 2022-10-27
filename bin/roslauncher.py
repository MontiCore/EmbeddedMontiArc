#!/usr/bin/env python

import signal
import sys
import time
import numpy as np


from optparse import OptionParser
from rosconnector import RosConnector

TRAINING_MODE = 0
PLAY_MODE = 1
EVAL_MODE = 2

def signal_handler(sig, frame):
    print('Starting shutdown')
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    parser = OptionParser(usage='usage: %prog [options]')
    parser.add_option('-e', '--environment', dest='environment',
                      help='environment string')
    parser.add_option("-v", '--verbose', action="store_true", dest="verbose",
                      help='show logging information')
    parser.add_option("-q", '--quiet', action="store_false", dest="verbose",
                      help='no output')
    parser.add_option('-r', '--render', dest='render', type='int',
                      help='number of steps after the game is rendered')
    parser.add_option('-t', '--training', action='store_true', dest='training',
                      help='training mode')
    parser.add_option('-p', '--play', action='store_true', dest='play',
                      help='play mode')
    parser.add_option('--eval', action='store_true', dest='eval',
                      help='evaluation mode')
    parser.add_option('-c', '--continuous', action='store_true',
                      dest='continuous', help='continuous environment')

    options, args = parser.parse_args()

    if not options.environment:
        parser.error('Enviroment string not given')

    mode_options = 0
    mode = TRAINING_MODE
    if options.training:
        mode = TRAINING_MODE
        mode_options += 1
    if options.play:
        mode = PLAY_MODE
        mode_options +=1
    if options.eval:
        mode = EVAL_MODE
        mode_options += 1
    if mode_options > 1:
        parser.error('more than one mode is not allowed')

    verbose = options.verbose
    render = 0 if not options.render else options.render
    continuous = True if options.continuous else False
    sample_games = 100 

    connector = RosConnector(options.environment, verbose, render, continuous)

    if mode == PLAY_MODE:
        time.sleep(8)
        connector.reset()
        while not connector.is_terminated or connector.in_reset:
            time.sleep(1)
    elif mode == EVAL_MODE:
        print('Start Evaluation Mode')
        time.sleep(5)
        print('Sample from {} games'.format(sample_games))
        total_socres = np.zeros((sample_games,), dtype='float32')
        for g in range(sample_games):
            connector.reset()
            while not connector.is_terminated:
                time.sleep(0.5)
    else: #training mode
        while True:
            time.sleep(0.5)
    connector.shutdown()        