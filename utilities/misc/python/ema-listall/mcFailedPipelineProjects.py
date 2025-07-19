def print_FailedPipelineProjects(group, abs):
  subgroups = group.subgroups
  abspath = abs + group.path +"/"
  for g in subgroups.list(all=True):
    current_group = gl.groups.get(g.id)
    for p in current_group.projects.list(all=True):
      project = gl.projects.get(p.id)
      pipelines = project.pipelines.list(ref='master')
      if not pipelines == []:
        pipeline = pipelines[0]
        if (pipeline.status == "failed"):
          print(abspath + g.path + "/" + p.path + "    " + str(
            p.id) + "    " + "Pipelinestatus: " + pipeline.status)
    print_FailedPipelineProjects(current_group, abspath)

import gitlab
import sys

gl=gitlab.Gitlab('https://git.rwth-aachen.de/', private_token=str(sys.argv[1]))

gl.auth()

#mc group id
mc = gl.groups.get(437)

print_FailedPipelineProjects(mc, "https://git.rwth-aachen.de/monticore/")

