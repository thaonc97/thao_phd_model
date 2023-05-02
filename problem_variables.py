import config as cfg

class Node:
    def __init__(self,coor,task, node_type):
        self.coor = coor
        self.task_idx = task
        self.node_type = node_type
        
    def __repr__(self):
        return f" {cfg.NODE_TYPE[self.node_type]} {self.coor} of task {self.task_idx}"


class Task:
    def __init__(self,pickup_coor, dest_coor,robots_required:int,index):
        self.pickup_coor = pickup_coor
        self.dest_coor = dest_coor
        self.n = robots_required
        if index is not None:
            self.index = index
        else:
            self.index = -1
    
    def __repr__(self):
        return f"Task {self.index},{self.pickup_coor},{self.dest_coor},{self.n}"
    
    def expand_nodes(self) -> tuple:
        """
        Use in expanded model
        """
        expanded_pickups = [Node(self.pickup_coor,self.index, 1) for _ in range(self.n)]
        expanded_dests = [Node(self.dest_coor,self.index, 2) for _ in range(self.n)]
        return (expanded_pickups, expanded_dests)