# (c) https://github.com/MontiCore/monticore  
#!/usr/bin/python

def getSpacesString():
    return '&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp'

def getStatus(status):
    if status == "failed":
        return '<div class=\'failedStatus\'>Failed</div>'
    elif status == "success":
        return '<div class=\'successStatus\'>Success</div>'
    else:
        return '<div class=\'noneStatus\'>' + status + '</div>'

def writeMainPipeline(pipelines, file, projectName):
    url = "https://git.rwth-aachen.de/" + projectName
    status = '<div class=\'noneStatus\'>None</div>'
    if not pipelines == []:
        status = getStatus(pipelines[0].status)
    file.write('\t\t"name": "<div class=\'depthImage0\'>' + 2*getSpacesString() + '<a class=\'ghLink\' href=\'' + url + '\' target=\'_blank\' rel=\'noopener\'>' + projectName + '</a></div>",\n' +
               '\t\t"status": "' + status + '",\n' +
               '\t\t"project": "' + projectName + '",\n' +
               '\t\t"branch": "main",\n')

def writeMasterPipeline(pipelines, file, projectName):
    file.write('\t\t\t{\n')
    if not pipelines == []:
        pipeline = pipelines[0]
        file.write( '\t\t\t\t"name": "<div class=\'atomicImage1\'>' + 3*getSpacesString() + '<a class=\'ghLink\' href=\'' + pipeline.web_url + '\' target=\'_blank\' rel=\'noopener\'>master</a></div>",\n' +
                    '\t\t\t\t"status": "' + getStatus(pipeline.status) + '",\n')
    else:
        file.write( '\t\t\t\t"name": "<div class=\'atomicImage1\'>' + 3*getSpacesString() + 'master</div>",\n' +
                    '\t\t\t\t"status": "None",\n')
    file.write(     '\t\t\t\t"project": "' + projectName + '",\n' +
                    '\t\t\t\t"branch": "master",\n' +
                    '\t\t\t\t"ChildData": []\n' +
                    '\t\t\t}')

def writeBranchPipeline(pipelines, file, projectName):
    if not pipelines == []:
        pipeline = pipelines[0]
        file.write( ',\n\t\t\t{\n' +
                    '\t\t\t\t"name": "<div class=\'atomicImage1\'>' + 3*getSpacesString() + '<a class=\'ghLink\' href=\'' + pipeline.web_url + '\' target=\'_blank\' rel=\'noopener\'>' + pipeline.ref + '</a></div>",\n'
                    '\t\t\t\t"status": "' + getStatus(pipeline.status) + '",\n' +
                    '\t\t\t\t"project": "' + projectName + '",\n' +
                    '\t\t\t\t"branch": "' + pipeline.ref + '",\n' +
                    '\t\t\t\t"ChildData": []\n' +
                    '\t\t\t}')

def print_ProjectPipelines(gl, group, file):
    global firstProject, firstBranch
    subgroups = group.subgroups
    for g in subgroups.list(all=True):
        current_group = gl.groups.get(g.id)
        for p in current_group.projects.list(all=True):
            project = gl.projects.get(p.id)
            if project.pipelines.list(all=True) != []:
                if not firstProject:
                    file.write(',\n')
                file.write('\t{\n')
                writeMainPipeline(project.pipelines.list(ref='master'), file, project.path_with_namespace)
                file.write('\t\t"ChildData": [\n')
                writeMasterPipeline(project.pipelines.list(ref='master'), file, project.path_with_namespace)
                for branch in project.branches.list():
                    if not branch.name == 'master':
                        writeBranchPipeline(project.pipelines.list(ref=branch.name), file, project.path_with_namespace)
                file.write('\n\t\t]\n\t}')
                firstProject = False
        print_ProjectPipelines(gl, current_group, file)

def initFile(output):
    output = output.replace('\\','/')
    if "/" in output:
        dir = output[0:output.index("/")]
        if not os.path.isdir(dir):
            os.makedirs(dir)
    file = open(output, 'w')
    file.write('[\n')
    return file

def endFile(file):
    file.write(']')
    file.close()

def main(output, access_token):
    gl = gitlab.Gitlab('https://git.rwth-aachen.de/', private_token=access_token)
    gl.auth()
    ema = gl.groups.get(9834)

    file = initFile(output)
    print_ProjectPipelines(gl, ema, file)
    endFile(file)

import gitlab, os, sys

firstProject = True
firstBranch = True
output = str(sys.argv[1])
access_token = str(sys.argv[2])
main(output, access_token)
