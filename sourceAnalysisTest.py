import Config
import sourceAnalysis
import yaml

from sourceAnalysis import findLargeFilesInHistory

config = Config.Config("config.yaml")
dr = sourceAnalysis.clone_and_scan(config)

data = yaml.safe_load(open("architecture.yaml"))
for repoID in data.keys():
    print()
    print(data[repoID]["Name"])
    # findLargeFilesInHistory("repos/" + data[repoID]["Name"])
