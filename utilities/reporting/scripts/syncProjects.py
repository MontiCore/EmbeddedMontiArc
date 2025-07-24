# (c) https://github.com/MontiCore/monticore  
def syncProjects(gl, group, gitlabToken):
    subgroups = group.subgroups
    for g in subgroups.list(all=True):
        current_group = gl.groups.get(g.id)
        for p in current_group.projects.list(all=True):
            project = gl.projects.get(p.id)
            syncProject(project, gitlabToken)
        syncProjects(gl, current_group, gitlabToken)


def syncProject(project, gitlabToken):
    repo_git_url = 'https://oauth2:' + gitlabToken + '@git.rwth-aachen.de/' + project.path_with_namespace + '.git'
    if os.path.isdir(project.name):
        print('Pulling ' + project.path_with_namespace)
        os.chdir(project.name)
        os.system('git pull ' + repo_git_url + ' --quiet')
        os.chdir('..')
    else:
        print('Cloning ' + project.path_with_namespace)
        os.system('git clone ' + repo_git_url + ' --quiet')

def main(gitUrl, groupID, outDir, accessToken):
    gl = gitlab.Gitlab(gitUrl, private_token=accessToken)
    gl.auth()
    ema = gl.groups.get(groupID)

    if not os.path.isdir(outDir):
        os.mkdir(outDir)
    os.chdir(outDir)
    os.environ['GIT_TERMINAL_PROMPT'] = '0'
    syncProjects(gl, ema, accessToken)
    os.chdir('..')

import gitlab
import sys
import os

main('https://git.rwth-aachen.de/', 9834, str(sys.argv[1]), str(sys.argv[2]))
