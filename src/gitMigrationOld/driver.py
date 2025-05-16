from src.gitMigrationOld.GitlabSource import Gitlab


def create_migration_config():
    Gitlab.create_config_file()


def scan(config):
    gitlab = Gitlab(config.url, config.sourceToken, config.repoIDS)
    gitlab.scan()


def clone(config):
    gitlab = Gitlab(config.url, config.sourceToken, config.repoIDS)
    gitlab.clone()


def clone_and_scan(config):
    gitlab = Gitlab(config.url, config.sourceToken, config.repoIDS)
    gitlab.clone()
    print()
    gitlab.scan()
