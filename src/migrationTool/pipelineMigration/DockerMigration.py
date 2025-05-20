import logging
import os
import gitlab
from rich.prompt import Confirm

from migrationTool.migration_types import Architecture, Config

logger = logging.getLogger(__name__)


class DockerMigration:
  def __init__(self, architecture: Architecture, config: Config, path: str):
    """
    Initializes the DockerMigration class with the given path.
    :param path: Path to the architecture file
    """
    self.path = path
    self.architecture = architecture
    self.config = config
    self.nativeImage = set()
    self.notNativeImage = set()
    self.migratedImages = self.read_previously_migrated_docker_images()
    self.newImages = self.add_images_being_migrated()
    self.dontMigrate = set()
    self.gl = gitlab.Gitlab(self.config.url, private_token=self.config.sourceToken)
    self.gl.auth()

  def add_images_being_migrated(self):
    """
    Changes the image names in the pipeline to the updated ones.
    :param pipeline: The pipeline object
    :param architecture: The architecture object
    :param repoID: The ID of the repository
    """
    # Get the full image names from the architecture
    repoIDS = self.config.repoIDS
    newImages = {}
    if type(repoIDS) == str:
      repoIDS = [repoIDS]
    for repoID in repoIDS:
      repo = self.architecture.get_repo_by_ID(repoID)
      if not repo.images:
        continue
      for image in repo.images:
        if not image.startswith(":"):
          url = ("registry." + self.config.url.replace("https://",
                                                       "") + repo.namespace + "/" + repo.name + "/" + image).lower()
          newImages[url] = ("ghcr.io/" + self.config.targetUser.lower() + "/" + repo.name + "/" + image)
        else:
          url = ("registry." + self.config.url.replace("https://",
                                                       "") + repo.namespace + "/" + repo.name + image).lower()
          newImages[url] = ("ghcr.io/" + self.config.targetUser.lower() + "/" + repo.name + "/" + image)
    return newImages

  def write_images_being_migrated(self):
    """
    Write the migrated docker images to the config file
    :param dockerImages: List of migrated docker images
    """
    with open(self.path, "a") as file:
      for original_url, new_url in self.newImages.items():
        if original_url not in self.migratedImages.keys():
          if new_url not in self.nativeImage:
            file.write(f"{original_url};{new_url};n\n")
          else:
            file.write(f"{original_url};{new_url};y\n")

  def read_previously_migrated_docker_images(self):
    """
    Read the migrated docker images from the config file
    :return: List of migrated docker image URLs
    """
    dockerImages = {}
    if not os.path.isfile(self.path):
      return dockerImages
    try:
      with open(self.path, "r") as file:
        lines = file.readlines()
      for line in lines:
        original_url, new_url, native = line.strip("\n").split(";")
        dockerImages[original_url] = new_url
        if native == "y":
          self.nativeImage.update(new_url)
    except:
      pass
    return dockerImages

  def get_new_Image(self, image: str) -> tuple[str, str]:
    """
    Returns the new image URL for the given image.
    :param image: Image URL
    :return: New image URL or original image URL if not found
    """
    if ":" not in image:
      image = image + ":latest"
    if image in self.migratedImages:
      return "", self.migratedImages[image]
    elif image in self.newImages.keys():
      image_new = self.newImages[image]
      if image_new not in self.nativeImage and image_new not in self.notNativeImage:
        print()
        if Confirm.ask(f"Can the image {image} natively be used in a github action?"):
          self.nativeImage.add(image_new)
        else:
          self.notNativeImage.add(image_new)
      return "", image_new
    else:
      if image.startswith("registry." + self.config.url.replace("https://", "")):
        print()
        if Confirm.ask(
            f"Image {image} is needed for the pipeline, but is not part of the current or a previous migration. Do "
            f"you want to migrate it?"):
          cleaned_image = image.split("/")[1:-1]
          image_repo = self.resolve_image_path_to_repo(cleaned_image)
          if image_repo:
            print(f"The image is saved in the repo {image_repo.name} and will automatically be migrated in future "
                  f"migrations.")
            image_new = "ghcr.io/" + self.config.targetUser.lower() + "/" + image_repo.name.lower() + "/" + \
                        image.split("/")[-1]
            self.newImages[image] = image_new
            if Confirm.ask(f"Can it natively be used in a github action?"):
              self.nativeImage.add(image_new)
            else:
              self.notNativeImage.add(image_new)
            return image_repo.name, image
        else:
          self.dontMigrate.add(image)
          logger.warning(f"Image {image} will not be migrated.")
          if Confirm.ask(f"Can the image {image} natively be used in a github action?"):
            self.nativeImage.add(image)
          else:
            self.notNativeImage.add(image)
          return "", image
      logger.warning(f"Could not update image: {image}")
      return "", image

  def dfs_resolve(self, group, remaining_parts, full_path):
    if not remaining_parts:
      return

    next_part = remaining_parts[0]

    # Check subgroups
    subgroups = group.subgroups.list(all=True)
    for subgroup in subgroups:
      if subgroup.path.lower() == next_part.lower():
        return self.dfs_resolve(self.gl.groups.get(subgroup.id), remaining_parts[1:], full_path)

    # Check projects at this group level
    projects = group.projects.list(all=True)
    for project in projects:
      if project.path.lower() == next_part.lower():
        logger.info(f"Found project: {project.name} in group: {group.name} for image: {full_path}")
        return project

  def resolve_image_path_to_repo(self, image_path):
    # path_parts = image_path.strip("/").split("/")
    path_parts = image_path
    # Search for top-level group
    root_groups = self.gl.groups.list(search=path_parts[0])
    root_group = None

    for g in root_groups:
      if g.path.lower() == path_parts[0].lower():
        root_group = self.gl.groups.get(g.id)
        break

    if not root_group:
      logger.warning(f"Could not find root group for image: {image_path}")
      return

    return self.dfs_resolve(root_group, path_parts[1:], image_path)
