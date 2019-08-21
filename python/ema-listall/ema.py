def print_projects(group, abs):
  print("\r\n")
  subgroups = group.subgroups
  
  abspath = abs + group.path +"/"
  for g in subgroups.list():
    print(abspath + g.path + "    " + str(g.id))
    current_group = gl.groups.get(g.id)  
    for p in current_group.projects.list():
      print(abspath + g.path +"/"+ p.path +"    " + str(p.id))
    print_projects(current_group, abspath)

import gitlab

gl=gitlab.Gitlab('https://git.rwth-aachen.de/', private_token='e7czjh4RiC3yh4RuxyLy')

gl.auth()

#ema group id
ema = gl.groups.get(9834)

print_projects(ema, "https://git.rwth-aachen.de/monticore/")

