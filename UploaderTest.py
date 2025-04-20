from repoMigration import GithubUploader
import Config

config = Config.Config("config.yaml")
Uploader = GithubUploader.GithubUploader(config.targetToken, config.sourceToken)
print(Uploader.listPrivateRepos())
for repoID in config.repoIDS:
    print("----------------")
    print("Uploading repo: " + repoID)
    Uploader.uploadRepo(repoID)
