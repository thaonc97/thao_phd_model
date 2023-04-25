import copy
import csv
from docplex.mp.model import Model
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import pandas as pd
import time

import config as cfg

seed_num =  cfg.rnd_seed
rnd = np.random
rnd.seed(seed_num)

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
        expanded_pickups = [Node(self.pickup_coor,self.index, 1) for _ in range(self.n)]
        expanded_dests = [Node(self.dest_coor,self.index, 2) for _ in range(self.n)]
        return (expanded_pickups, expanded_dests)
        pass

def gen_tasks(num_tasks,max_robots, coor_range = 500, no_delivery = False):
    """
    if no-delivery = true then every tasks has their pick up node = their destination node.
    """
    tasks = []
    for i in range(1,num_tasks + 1): # index start from 1 because 0 is the depot
        pickup_coor = rnd.uniform(0, coor_range,2)
        if no_delivery ==True:
            dest_coor = pickup_coor
        else:
            dest_coor = rnd.uniform(0, coor_range,2)
        if max_robots !=1:
            total_robots_required = rnd.randint(1,max_robots+1)
        else:
            total_robots_required = 1
        cur_tasks = Task(pickup_coor, dest_coor, total_robots_required, i)
        tasks.append(cur_tasks)
    
    return tasks

def gen_depot(coor_range = 500):
    coor = rnd.uniform(0, coor_range,2)
    depot = Node(coor, 0, 0)
    return depot

def construct_v_prime(depot,tasks):
    v_prime = [depot]
    for task in tasks:
        cur_pickups, cur_dests = task.expand_nodes()
        v_prime += cur_pickups
        v_prime += cur_dests
    
    return v_prime

def add_node_index(v_prime):
    
    total_nodes = len(v_prime)
    for i in range(total_nodes):
        v_prime[i].index = i

    return v_prime

def update_connection(v_prime):
    """
    Note that v_prime must have indicies already
    """
    depot = v_prime[0]
    depot.connection = [node.index for node in v_prime if node.index != depot.index and node.node_type == 1]# depot
    for cur_node in v_prime[1:]:
        cur_task_idx =  cur_node.task_idx
        cur_type = cur_node.node_type
        if cur_type == 1: #pickup
            cur_node.connection = [node.index for node in v_prime if node.task_idx == cur_task_idx and node.node_type ==2]
        elif cur_type == 2: #destination
            cur_node.connection = [node.index for node in v_prime if node.task_idx != cur_task_idx and node.node_type in [0,1]]
    return v_prime

def construct_e_prime(v_prime_connected):
    """
    Parameters
    ----------
    v_prime_connected: set of v_prime with info of connection
    """
    total_nodes = len(v_prime_connected)
    e_prime = {}
    e_prime ={
        i: v_prime_connected[i].connection for i in range(total_nodes)
    }
    return e_prime

def cal_dist(node1,node2):
    dist = np.linalg.norm(node1.coor - node2.coor)
    return dist

def create_cost_dict(connected_v_prime,node_idx_mapper):
    node_idx_mapper = {node.index: node for node in connected_v_prime}
    c = {}
    for cur_node in connected_v_prime:
        cur_idx = cur_node.index
        dist = {
            node_idx_mapper[node].index:cal_dist(cur_node,node_idx_mapper[node]) for node in cur_node.connection
        }
        c[cur_idx] = dist
    return c

def plot_map(depot, tasks):
    plt.scatter(depot.coor[0], depot.coor[1],c= 'C3', label = 'depot')
    for task in tasks:
        pickup_coor = task.pickup_coor
        dest_coor = task.dest_coor
        plt.scatter(pickup_coor[0], pickup_coor[1],c = 'C0',)
        plt.scatter(dest_coor[0], dest_coor[1], c ='C1',)
        xs = [pickup_coor[0],dest_coor[0]]
        ys = [pickup_coor[1],dest_coor[1]]
        plt.plot(xs, ys, c = cfg.PATH_COLOR[task.n])
        
    robots_required = set([task.n for task in tasks])
    legends_path ={f"{i} robot(s)":Line2D([0],[0], color = f"C{i}",lw = 2) for i in robots_required}
    legends_node = {
        "depot" : Line2D([0], [0], marker='o', color='w',markerfacecolor='C3', markersize= 8),
        "pickup" : Line2D([0], [0], marker='o', color='w',markerfacecolor='C0', markersize= 8),
        "dest." : Line2D([0], [0], marker='o', color='w',markerfacecolor='C1', markersize= 8),
    }
    legends = dict(legends_path,**legends_node)
    plt.legend(legends.values(), legends.keys())
    plt.plot()
    
def get_data_from_csv(path):
    df = pd.read_csv(path)
    data_dict = df.to_dict(orient = 'records')
    depot_raw = data_dict.pop(0)
    depot = Node(np.array([depot_raw['x_pickup'], depot_raw['y_pickup']]),0,0)
    for task in data_dict:
        task['pickup_coor'] = np.array([task['x_pickup'],task['y_pickup']])
        task['dest_coor'] = np.array([task['x_dest'],task['y_dest']])
        to_remove = ('x_pickup', 'y_pickup', 'x_dest', 'y_dest')
        [task.pop(field) for field in to_remove]
    tasks = [Task(**data) for data in data_dict]
    return depot, tasks