from dgl.data.utils import load_graphs

class GraphDataAccess:
    def __init__(self, datasource):
        self._datasource_ = datasource
        
    def execute(self):
        graph = load_graphs(self._datasource_)[0][0]
        return graph