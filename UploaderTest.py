import yaml

#from combinedTest import architecture
from repoMigration import GithubUploader
import Config

config = Config.Config("config.yaml")
Uploader = GithubUploader.GithubUploader(config.targetToken, config.sourceToken)

architecture = yaml.safe_load(open("architecture.yaml"))

print(Uploader.listPrivateRepos())
for repoID in config.repoIDS:
    secrets = {}
    for secretName, secretValues in architecture[repoID]["Secrets"].items():
        if secretValues["Secret"].lower() == "y":
            if secretName not in secrets.keys() and secretValues["Value"] != "Please add a value":
                secrets[secretName] = secretValues["Value"]
    print("----------------")
    print("Uploading repo: " + repoID)
    Uploader.uploadRepo(repoID,secrets)
    Uploader.dockerImageMigration(architecture,repoID)
