import copy
import numpy as np

from helpers import create_cost_dict
from problem_variables import Node, Task, Processed,Raw


def construct_v_prime(depot,tasks):
    v_prime = [depot]
    for task in tasks:
        cur_pickups, cur_dests = task.expand_nodes()
        v_prime += cur_pickups
        v_prime += cur_dests
    
    return v_prime

def add_node_index(nodes):
    
    total_nodes = len(nodes)
    for i in range(total_nodes):
        nodes[i].index = i

    return nodes

def update_connection(nodes):
    """
    Note that nodes must have indicies already
    """
    depot = nodes[0]
    depot.connection = [node.index for node in nodes if node.index != depot.index and node.node_type == 1]# depot
    for cur_node in nodes[1:]:
        cur_task_idx =  cur_node.task_idx
        cur_type = cur_node.node_type
        if cur_type == 1: #pickup
            cur_node.connection = [node.index for node in nodes if node.task_idx == cur_task_idx and node.node_type ==2]
        elif cur_type == 2: #destination
            cur_node.connection = [node.index for node in nodes if node.task_idx != cur_task_idx and node.node_type in [0,1]]
    return nodes

def construct_edges(nodes_connected):
    """
    Parameters
    ----------
    nodes_connected: set of nodes with info of connection
    """
    total_nodes = len(nodes_connected)
    edges ={
        i: nodes_connected[i].connection for i in range(total_nodes)
    }
    return edges


#flow
def construct_v(depot,tasks): # for flow model
    vs = [depot]
    for task in tasks:
        pickup_node = Node(task.pickup_coor,task.index,1)
        dest_node = Node(task.dest_coor,task.index,2)
        
        vs.append(pickup_node)
        vs.append(dest_node)
    sink = copy.deepcopy(depot)
    sink.task_idx = tasks[-1].index + 1
    sink.node_type = 3
    vs.append(sink)
    return vs

def cal_capacity(edge_jobs,edge_travels,node_idx_mapper,tasks):
    capac = {}
    for edge in edge_jobs:
        pu_node = edge[0]
        task_idx = node_idx_mapper[pu_node].task_idx
        cur_task = [task for task in tasks if task.index == task_idx][0]
        cur_capac = cur_task.n
        capac[edge] = cur_capac
    for edge in edge_travels:
        capac[edge] = 1
    return capac

def update_connection_flow(v_prime): #for flow model
    """
    Note that v_prime must have indicies already
    """
    depot = v_prime[0]
    depot.connection = [node.index for node in v_prime if (node.node_type == 1 or node.node_type ==3)]# depot
    for cur_node in v_prime[1:]:
        cur_task_idx =  cur_node.task_idx
        cur_type = cur_node.node_type
        if cur_type == 1: #pickup
            cur_node.connection = [node.index for node in v_prime if node.task_idx == cur_task_idx and node.node_type ==2]
        elif cur_type == 2: #destination
            cur_node.connection = [node.index for node in v_prime if node.task_idx != cur_task_idx and node.node_type in [1,3]]
    sink = v_prime[-1]
    sink.connection = []
    return v_prime

def construct_edge_travels(edges,node_idx_mapper): #for flow based model
    edge_travels = []
    for node in edges:
        if node_idx_mapper[node].node_type ==0: #depot
            edge_travels +=[(node,to_node) for to_node in edges[node]]
        if node_idx_mapper[node].node_type ==2: #destination
            edge_travels +=[(node,to_node) for to_node in edges[node]]
    return edge_travels

def construct_edge_jobs(edges,node_idx_mapper):  #for flow based model
    edge_jobs = []
    for node in edges:
        if node_idx_mapper[node].node_type ==1: #pickup
            edge_jobs +=[(node,to_node) for to_node in edges[node]]
    return edge_jobs

#VRP ver 2
def expand_tasks(tasks: list[Task]):
    expanded_tasks = []
    i = min([task.index for task in tasks])
    for task in tasks:
        for _ in range(task.n):
            expanded_cur_task = copy.deepcopy(task)
            expanded_cur_task.is_expand = True
            expanded_cur_task.origin_idx = task.index
            expanded_cur_task.n = 1
            expanded_cur_task.index = i
            i = i + 1
            expanded_tasks.append(expanded_cur_task)
    return expanded_tasks

def process_vrp_sumofcosts(raw):
    depot = raw.depot
    tasks = raw.tasks
    v_prime = construct_v_prime(depot,tasks)
    add_node_index(v_prime)
    v_prime = update_connection(v_prime)
    e_prime = construct_edges(v_prime)
    processed_data = Processed(depot,tasks,v_prime,e_prime)
    return processed_data

def process_flow(raw):
    depot = raw.depot
    tasks = raw.tasks
    vs = construct_v(depot,tasks)
    add_node_index(vs)
    vs = update_connection_flow(vs)
    es = construct_edges(vs)
    processed_data = Processed(depot,tasks,vs,es)
    return processed_data

def process_vrp_makespan(raw):
    depot = raw.depot
    tasks = raw.tasks
    v_prime = construct_v_prime(depot, tasks)
    sink = copy.deepcopy(depot)
    sink.task_idx = tasks[-1].index + 1
    sink.node_type = 3
    v_prime.append(sink)
    add_node_index(v_prime)
    v_prime = update_connection(v_prime)
    e_prime = construct_edges(v_prime)
    processed_data = Processed(depot,tasks,v_prime,e_prime)
    return processed_data

def process_vrp_ver2(raw : Raw):
    raw_depot = raw.depot
    depot_coor = raw_depot.coor
    depot = Task(depot_coor,depot_coor,0,0,0)
    tasks = raw.tasks
    expanded_tasks = expand_tasks(tasks)
    processed_data = Processed(depot, expanded_tasks, None, None)
    return processed_data

def process_vrp_ver2_makespan(raw: Raw):
    raw_depot = raw.depot
    depot_coor = raw_depot.coor
    depot = Task(depot_coor,depot_coor,0,0,0)
    tasks = raw.tasks
    expanded_tasks = expand_tasks(tasks)
    sink = copy.deepcopy(depot)
    sink.index = expanded_tasks[-1].index + 1
    sink.origin_idx = tasks[-1].index + 1
    expanded_tasks.append(sink)
    processed_data = Processed(depot, expanded_tasks, None, None)
    return processed_data

PROCESS_METHOD ={
    1: process_vrp_sumofcosts,
    2: process_flow,
    3: process_vrp_makespan,
    4: process_vrp_ver2,
    5: process_vrp_ver2_makespan
}

def process(raw, method):
    process_method = PROCESS_METHOD[method]
    processed_data = process_method(raw)
    return processed_data
