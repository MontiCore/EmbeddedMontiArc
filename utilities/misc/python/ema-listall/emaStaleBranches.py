def print_Branches(group, abs):
  subgroups = group.subgroups
  abspath = abs + group.path +"/"
  for g in subgroups.list(all=True):
    current_group = gl.groups.get(g.id)
    for p in current_group.projects.list(all=True):
      project = gl.projects.get(p.id)
      branches = project.branches.list()
      if not branches == []:
        for branch in branches:
          if branch.name == "master" or branch.name == "main" or branch.name == "dev" or branch.name == "develop":
            continue
          auth_date = parse(branch.commit['authored_date'])
          age = datetime.now(timezone.utc) - auth_date
          
          if (age.days>90):
            print(abspath + g.path + "/" + p.path + "    " + str(p.id) + "    " + "Branch: " + str(branch.name))
    print_Branches(current_group, abspath)

import gitlab
import sys
import datetime
from dateutil.parser import parse
from datetime import datetime, timezone

gl=gitlab.Gitlab('https://git.rwth-aachen.de/', private_token=str(sys.argv[1]))

gl.auth()

#ema group id
ema = gl.groups.get(437)

print_Branches(ema, "https://git.rwth-aachen.de/")

