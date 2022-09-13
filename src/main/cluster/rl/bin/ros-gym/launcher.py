# (c) https://github.com/MontiCore/monticore
import signal
import sys
import time
import numpy as np

from rosgym import RosGymConnector
from optparse import OptionParser

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
                      help='gym environment string')
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
        parser.error('Environment string not given')

    mode_options = 0
    #mode = TRAINING_MODE
    if options.training:
        mode = TRAINING_MODE
        mode_options += 1
    if options.play:
        mode = PLAY_MODE
        mode_options += 1
    if options.eval:
        mode = EVAL_MODE
        mode_options += 1
    if mode_options > 1:
        parser.error('Please select only one mode')

    verbose = options.verbose
    render = 0 if not options.render else options.render
    continuous = True if options.continuous else False
    sample_games = 10

    connector = RosGymConnector(options.environment, verbose, render,
                                continuous)

    if mode == PLAY_MODE:
        time.sleep(8)
        connector.reset()
        #while not connector.is_terminated or connector.in_reset:
        while True:
            time.sleep(1)
    elif mode == EVAL_MODE:
        print('Start Evaluation Mode')
        time.sleep(5)
        print('Sample from {} games'.format(sample_games))
        total_scores = np.zeros((sample_games,), dtype='float32')
        for g in range(sample_games):
            connector.reset()
            while not connector.is_terminated:
                time.sleep(0.5)
            total_scores[g] = connector.last_game_score
            print('Game {}/{} Score: {}'.format(
                g+1, sample_games, total_scores[g]))
        print('---- Statistics for {} games played ----'.format(sample_games))
        print('Average Score: {}'.format(total_scores.mean()))
        print('Deviation: {}'.format(total_scores.std()))
        print('Min: {}'.format(total_scores.min()))
        print('Max: {}'.format(total_scores.max()))
    else:
        while True:
            time.sleep(0.5)
    connector.shutdown()
