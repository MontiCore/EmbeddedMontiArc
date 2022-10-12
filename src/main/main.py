from local import main_cluster as cluster
from local import main_local as local
from local import cluster_agent as agent

from optparse import OptionParser
from local.tools import winscp

if __name__ == "__main__":
    parser = OptionParser(usage='usage: %prog [options]')
    parser.add_option('-i', '--install-remote',action="store_true", dest='install',
                      help='copies the cluster code to the hpc cluster')
    parser.add_option("-t", '--train', action="store_true", dest="train",
                      help='start the training')
    parser.add_option("-l", '--local', action="store_true", default=False, dest="local",
                      help='train on the local pc instead of the cluster')
    parser.add_option("-e", '--execute-agent', action="store_true", dest="execute_agent",
                      help='execute the current for a given constraint')

    options, args = parser.parse_args()

    if options.install:
        cluster.install()
    elif options.train:
        if not options.local:
            cluster.train()
        else:
            print("Currently not working please train on the cluster for now")
            #local.train()
    elif options.execute_agent:
        agent.execute()