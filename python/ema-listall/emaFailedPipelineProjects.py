def print_FailedPipelineProjects(group, abs):
  # print("\r\n")
  subgroups = group.subgroups

  abspath = abs + group.path +"/"
  for g in subgroups.list(all=True):
    current_group = gl.groups.get(g.id)
    # print(abspath + g.path + "    " + str(g.id) + "    (" +str(len(current_group.projects.list(all=True))) + " projects)")
    for p in current_group.projects.list(all=True):
      lastMasterPipelineStatus = "-"
      project = gl.projects.get(p.id)
      pipelines = project.pipelines.list(all=True)
      for pipeline in pipelines:
        if pipeline.ref == "master":
          lastMasterPipelineStatus = pipeline.status
          break
      if(lastMasterPipelineStatus == "failed"):
        print(abspath + g.path +"/"+ p.path + "    " + str(p.id) + "    " + "Pipelinestatus: " + lastMasterPipelineStatus)
    print_FailedPipelineProjects(current_group, abspath)

import gitlab

gl=gitlab.Gitlab('https://git.rwth-aachen.de/', private_token='BDwwZXqRkex-tx7GCRNz')

gl.auth()

#ema group id
ema = gl.groups.get(9834)

print_FailedPipelineProjects(ema, "https://git.rwth-aachen.de/monticore/")

