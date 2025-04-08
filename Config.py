import yaml


class Config:
    def __init__(self, path):
        data = yaml.safe_load(open(path))
        self.url = data['URL']
        self.sourceToken = data['SourceToken']
        self.targetToken = data['TargetToken']
        self.repoIDS = [str(i) for i in data['Repos']]
