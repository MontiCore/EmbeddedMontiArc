# (c) https://github.com/MontiCore/monticore  
import signal
import sys
import time
import ros_torcs_wrapper

from optparse import OptionParser


# Options
MSG_LEVEL = ros_torcs_wrapper.INFO
PLAY_MODE = False
ADJUST_ACTIONS = True


def signal_handler(sig, frame):
    print('Starting shutdown')
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    # parser = OptionParser(usage='usage: %prog [options]')
    # parser.add_option('--quiet', action="store_true", dest="quiet",
    #                  help='no output')
    # parser.add_option('--show', action='store_true', dest="debug",
    #                  help='Show debug information')
    # parser.add_option('--play', action='store_true', dest='play',
    #                   help='play mode')

    # options, args = parser.parse_args()

    # msg_level = ros_torcs_wrapper.INFO
    # if options.debug:
    #     msg_level = ros_torcs_wrapper.DEBUG
    # elif options.quiet:
    #     msg_level = ros_torcs_wrapper.NONE

    # play_mode = False if not options.play else True

    # msg_level = ros_torcs_wrapper.INFO
    connector = ros_torcs_wrapper.RosTorcsConnector(
        tanh_adjust=ADJUST_ACTIONS, msg_level=MSG_LEVEL, vision=True)

    if PLAY_MODE:
        for g in range(10):
            print('---- PLAY MODE ----')
            print('Make Reset...')
            connector.reset()
            print('TORCS started....')
            while not connector.is_terminated() or connector.in_reset():
                time.sleep(5.0)
    else:
        while True:
            time.sleep(0.5)
    connector.shutdown()
