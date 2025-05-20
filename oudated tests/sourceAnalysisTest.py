from src import gitMigrationOld
from src.types import Config
import yaml

config = Config.Config("../config.yaml")
dr = gitMigrationOld.clone_and_scan(config)

data = yaml.safe_load(open("../architecture.yaml"))
for repoID in data.keys():
    print()
    print(data[repoID]["Name"])
    # findLargeFilesInHistory("repos/" + data[repoID]["Name"])
