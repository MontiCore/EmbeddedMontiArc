import yaml

#from combinedTest import architecture
from repoMigration import GithubUploader
import Config

config = Config.Config("config.yaml")
Uploader = GithubUploader.GithubUploader(config.targetToken, config.sourceToken)

architecture = yaml.safe_load(open("architecture.yaml"))

print(Uploader.listPrivateRepos())
for repoID in config.repoIDS:
    print("----------------")
    print("Uploading repo: " + repoID)
    Uploader.uploadRepo(repoID)
    #Uploader.dockerImageMigration("monticore/EmbeddedMontiArc/generators/", architecture[repoID]["Name"], architecture[repoID]["DockerImages"])
