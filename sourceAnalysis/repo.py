from gitlab.v4.objects import artifacts


class Repo:
    def __init__(self, name: str, id: str, images: list[str], artifacts: list[str], path: str):
        self.name = name
        self.id = id
        self.images = images
        self.artifacts = artifacts
        self.location = path

    def __str__(self):
        Output = "---------"
        Output += f"Repo: {self.name}\n"
        Output += f"ID: {self.id}\n"
        Output += f"Images: {self.images}\n"
        Output += f"Artifacts: {self.artifacts}\n"
        Output += f"Location: {self.location}\n"
        return Output