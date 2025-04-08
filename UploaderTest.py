from repoMigration import GithubUploader
import Config

config = Config.Config("repos.yaml")
Uploader = GithubUploader.GithubUploader(config.targetToken)
print(Uploader.listPrivateRepos())
for repoID in config.repoIDS:
    print("----------------")
    print("Uploading repo: " + repoID)
    Uploader.uploadRepo(repoID)
