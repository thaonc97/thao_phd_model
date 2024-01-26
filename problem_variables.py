import copy

import config as cfg

class Node:
    def __init__(self,coor,task, node_type):
        self.coor = coor
        self.task_idx = task
        self.node_type = node_type
        
    def __repr__(self):
        return f" {cfg.NODE_TYPE[self.node_type]} {self.coor} of task {self.task_idx}"


class Task:
    def __init__(self,pickup_coor, dest_coor, process_time: int, robots_required:int,index):
        self.pickup_coor = pickup_coor
        self.dest_coor = dest_coor
        self.process_time = process_time
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
    
class Raw:
    def __init__(self,depot: Node, tasks :list[Task], *args, **kwargs):
        self.depot = depot
        self.tasks = tasks
    
    def unpack(self):
        depot = self.depot
        tasks = self.tasks
        
        return depot, tasks

class Processed(Raw):
    def __init__(self, depot,tasks :list[Task],nodes, edges):
        self.depot = depot
        self.tasks = tasks
        self.nodes = nodes
        self.edges = edges

    def set_nodes(self, nodes):
        self.node = nodes

    def set_edges(self, edges):
        self.edges = edges

    def unpack(self):
        depot = self.depot
        tasks = self.tasks
        nodes = self.nodes
        edges = self.edges
        return depot, tasks, nodes, edges
    

class SolvedInstance():
    
    def __init__(self, seed, tasks, total_robots, v_prime, e_prime, sol_result, solve_time,instance_num):
        self.seed = seed
        self.tasks = tasks
        self.total_robots = total_robots
        self.num_tasks = len(tasks)
        self.num_nodes = len(v_prime)
        self.edges = e_prime
        self.solve_time = solve_time
        self.gap = self._get_gap(sol_result)
        self.instance_num = instance_num
        
    def _get_solve_time(self,sol_result):
        pass

    def _get_gap(self, solve_result):
        return solve_result['gap']