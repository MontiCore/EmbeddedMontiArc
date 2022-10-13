import sys
from local import main_cluster as cluster
from local import local_agent as local_agent
from local import cluster_agent as cluster_agent

from optparse import OptionParser


def execute(options):
    if options.local:
        local_agent.execute()
    else:
        cluster_agent.execute()


if __name__ == "__main__":
    parser = OptionParser(usage='usage: python %prog [options] cmd')

    parser.set_usage("""usage: python %prog [options] COMMAND
Commands:
  install   install the remote part of this software on the hpc cluster
  train     start the training of the agent
  execute   execute the current agent
""")
    parser.add_option("-l", '--local', action="store_true", default=False, dest="local",
                      help='execute agent on the local pc instead of the cluster')

    options, args = parser.parse_args()

    if len(args) == 0:
        parser.print_help()
        sys.exit(0)

    commands = dict({
        "install": cluster.install,
        "train": cluster.train,
        "execute": execute
    })

    cmd = args[0]

    if cmd in commands.keys():
        commands[cmd](options)
    else:
        print(
            f'Unrecognized command {cmd}. Run `python main.py --help` for a list of available commands')
