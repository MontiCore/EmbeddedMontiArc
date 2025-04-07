from pipelineMigration import GitlabToGithub

gitlabFilePath = ".gitlab-ci.yml"
githubFilePath = ".main.yml"
GitlabToGithub(gitlabFilePath, githubFilePath, "pipeline", ["GitlabToken", "URL", "ID"])