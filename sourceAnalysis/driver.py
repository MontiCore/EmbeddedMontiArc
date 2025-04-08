from sourceAnalysis.GitlabSource import Gitlab

def createMigrationConfig():
    Gitlab.createConfigFile()

def scanRepos(config):
    gitlab = Gitlab(config.url, config.sourceToken, config.repoIDS)
    gitlab.scanRepos()

def cloneRepos(config):
    gitlab = Gitlab(config.url, config.sourceToken, config.repoIDS)
    gitlab.cloneRepos()

def scanAndCloneRepos(config):
    gitlab = Gitlab(config.url, config.sourceToken, config.repoIDS)
    gitlab.scanRepos()
    gitlab.cloneRepos()