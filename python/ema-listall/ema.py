def print_projects(group, abs):
  print("\r\n")
  subgroups = group.subgroups
  
  abspath = abs + group.path +"/"
  for g in subgroups.list(all=True):
    current_group = gl.groups.get(g.id)  
    print(abspath + g.path + "    " + str(g.id) + "    (" +str(len(current_group.projects.list(all=True))) + " projects)")
    for p in current_group.projects.list(all=True):
      print(abspath + g.path +"/"+ p.path +"    " + str(p.id))
    print_projects(current_group, abspath)

import gitlab

gl=gitlab.Gitlab('https://git.rwth-aachen.de/', private_token='kfyiVbKz-8V1rqn_bKCy')

gl.auth()

#ema group id
ema = gl.groups.get(9834)

print_projects(ema, "https://git.rwth-aachen.de/monticore/")

