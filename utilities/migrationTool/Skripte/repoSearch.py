import gitlab
import sys

# Configure your GitLab URL and personal access token
GITLAB_URL = "https://git.rwth-aachen.de/"
GITLAB_TOKEN = "glpat-1E9aKZussdM6-vaqwDGk"

# Authenticate with GitLab
gl = gitlab.Gitlab(GITLAB_URL, private_token=GITLAB_TOKEN)
gl.auth()


def dfs_resolve(group, remaining_parts, full_path):
  if not remaining_parts:
    return

  next_part = remaining_parts[0]

  # Check subgroups
  subgroups = group.subgroups.list(all=True)
  for subgroup in subgroups:
    if subgroup.path.lower() == next_part.lower():
      dfs_resolve(gl.groups.get(subgroup.id), remaining_parts[1:], full_path)
      return

  # Check projects at this group level
  projects = group.projects.list(all=True)
  for project in projects:
    if project.path.lower() == next_part.lower():
      # If this is the last part, it might be the project
      if len(remaining_parts) == 1:
        print(f"✅ Found project: {project.web_url}")
        return
      else:
        # This isn't supposed to happen in clean Docker image paths
        print(f"⚠️ Found project {project.web_url} mid-path — remaining: {remaining_parts[1:]}")
        return


def resolve_image_path_to_repo(image_path):
  path_parts = image_path.strip("/").split("/")

  # Search for top-level group
  root_groups = gl.groups.list(search=path_parts[0])
  root_group = None

  for g in root_groups:
    if g.path.lower() == path_parts[0].lower():
      root_group = gl.groups.get(g.id)
      break

  if not root_group:
    print(f"❌ Could not find root group: {path_parts[0]}")
    return

  dfs_resolve(root_group, path_parts[1:], image_path)


if __name__ == "__main__":
  image_path = "/monticore/embeddedmontiarc/generators/emadl2cpp/dockerimages/pytorch"
  resolve_image_path_to_repo(image_path)
