from platform import architecture

from repoMigration import GithubUploader
import Config
import yaml

print("Uploading repos to Github")
config = Config.Config("repos.yaml")
Uploader = GithubUploader.GithubUploader(config.targetToken, config.sourceToken)
architecture = yaml.safe_load(open("architecture.yaml"))

print(Uploader.listPrivateRepos())
for repoID in config.repoIDS:
    print("----------------")
    print("Uploading repo: " + repoID)
    repoArchitecture = architecture[repoID]
    Uploader.dockerImageMigration(repoArchitecture["Namespace"],repoArchitecture["Name"],repoArchitecture["DockerImages"])
    Uploader.uploadRepo(repoID)
