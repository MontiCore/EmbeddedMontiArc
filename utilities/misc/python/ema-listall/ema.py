def print_projects(group, abs):
  print("\r\n")
  subgroups = group.subgroups
  
  abspath = abs + group.path +"/"
  for g in subgroups.list(all=True):
    current_group = gl.groups.get(g.id)  
    print(abspath + g.path + "    " + str(g.id) + "    (" +str(len(current_group.projects.list(all=True))) + " projects)")
    for p in current_group.projects.list(all=True):
      project = gl.projects.get(p.id)
      try:
        try:
          file = project.files.get('src/license/se/license.txt', ref='master')
          if('*' in str(file.decode())):
            print(abspath + g.path +"/"+ p.path +"    " + str(p.id))
        except:
           pass
      except gitlab.exceptions.GitlabHttpError:
        pass
      
    print_projects(current_group, abspath)

import gitlab
import sys

gl=gitlab.Gitlab('https://git.rwth-aachen.de/', private_token=str(sys.argv[1]))

gl.auth()

#ema group id
ema = gl.groups.get(9834)

print_projects(ema, "https://git.rwth-aachen.de/monticore/")

