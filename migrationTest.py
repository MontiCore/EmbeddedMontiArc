from pipelineMigration import GitlabToGithub

gitlabFilePath = ".gitlab-ci.yml"
githubFilePath = ".main.yml"
GitlabToGithub(gitlabFilePath, "pipeline", ["GitlabToken", "URL", "ID"])