import copy
import numpy as np

from problem_variables import Node, Task


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


def process_vrp_sumofcosts(data):
    depot, tasks = data
    v_prime = construct_v_prime(depot,tasks)
    add_node_index(v_prime)
    v_prime = update_connection(v_prime)
    node_idx_mapper = {node.index: node for node in v_prime}
    c = create_cost_dict(v_prime,node_idx_mapper)
    e_prime = construct_edges(v_prime)
    return v_prime, e_prime


def process_flow_sumofcost(data):
    depot, tasks = data
    vs = construct_v(depot,tasks)
    add_node_index(vs)
    vs = update_connection_flow(vs)
    node_idx_mapper = {node.index: node for node in vs}
    c = create_cost_dict(vs,node_idx_mapper)
    es = construct_edges(vs)

