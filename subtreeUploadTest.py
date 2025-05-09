import yaml
from repoMigration import GithubUploader
import Config

config = Config.Config("config.yaml")
architecture = yaml.safe_load(open("architecture.yaml"))

dockerImages = []
secrets = {}
for reopId in architecture.keys():
    dockerImages.append(architecture[reopId]["DockerImages"])
    for secretName, secretValues in architecture[reopId]["Secrets"].items():
        if secretValues["Secret"].lower() == "y":
            if secretName not in secrets.keys() and secretValues["Value"] != "Please add a value":
                secrets[secretName] = secretValues["Value"]

Uploader = GithubUploader.GithubUploader(config.targetToken, config.sourceToken)
Uploader.dockerImageMigrationMonorepo(architecture,"subtreeTest")
Uploader.uploadMonoRepo("subtreeUploader",secrets ,"./repos/subtreeTest")