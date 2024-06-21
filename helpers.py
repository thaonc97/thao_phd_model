import networkx as nx
import numpy as np

import config as cfg
from problem_variables import Task, Node

def cal_dist(node1,node2):
    dist = np.linalg.norm(node1.coor - node2.coor)
    return dist

def cal_dist_task(task1, task2):
    dist = np.linalg.norm(task1.dest_coor - task2.pickup_coor)
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

def create_task_dist_dict(depot, tasks: list[Task]):
    c = {}
    depot_tasks = [depot] + tasks
    for cur_task in depot_tasks:
        dist = {
            task.index: cal_dist_task(cur_task, task) for task in depot_tasks
        }
        c[cur_task.index] = dist

    return c

def create_operate_dict(depot, tasks: list[Task]):
    c = {}
    depot_idx = depot.index
    c[depot_idx] = 0
    for task in tasks:
        c[task.index] = task.process_time
    return c

def create_original_task_mapper(expanded_tasks):
    orig_expanded = {}
    for task in expanded_tasks:
        cur_orig_idx = task.origin_idx
        cur_idx = task.index
        if cur_orig_idx not in orig_expanded:
            orig_expanded[cur_orig_idx] = [cur_idx]
        else:
            orig_expanded[cur_orig_idx].append(cur_idx)
    return orig_expanded

def create_expanded_original_task_mapper(expanded_tasks):
    orig_expanded = {
        task.index: task.origin_idx for task in expanded_tasks
    }
    return orig_expanded

def cal_big_m(nodes):
    node_idx_mapper = {node.index: node for node in nodes}
    c = create_cost_dict(nodes,node_idx_mapper)
    big_m = 0
    for node in c:
        try:
            big_m += max(c[node].values())
        except ValueError:
            pass
    return big_m


def create_cost_dict_wr2(raw_with_dist_travel):
    dist_matrix = raw_with_dist_travel.distance_matrix
    num_tasks = len(raw_with_dist_travel.tasks)
    c = {}
    for i in range(1,num_tasks+1):
        dist = {
            j:dist_matrix[i+num_tasks][j] for j in range(1,num_tasks + 1)
        }
        c[i] = dist

    return c

def h_equal(a,b,eps=1e-6):
    return abs(a-b) < eps