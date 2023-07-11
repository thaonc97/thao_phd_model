import numpy as np
import pandas as pd

import config as cfg
import helpers
from problem_variables import Task, Node, Raw


seed_num =  cfg.RND_SEED
rnd = np.random
rnd.seed(seed_num)

def _get_data_from_csv(path, depot_as_task: True):
    df = pd.read_csv(path)
    data_dict = df.to_dict(orient = 'records')
    depot_raw = data_dict.pop(0)
    depot = Node(np.array([depot_raw['x_pickup'], depot_raw['y_pickup']]),0,0)
    if depot_as_task:
        depot = Task(np.array([depot_raw['x_pickup'], depot_raw['y_pickup']]),np.array([depot_raw['x_dest'],depot_raw['y_dest']]),0,0,0)
    for task in data_dict:
        task['pickup_coor'] = np.array([task['x_pickup'],task['y_pickup']])
        task['dest_coor'] = np.array([task['x_dest'],task['y_dest']])
        to_remove = ('x_pickup', 'y_pickup', 'x_dest', 'y_dest')
        [task.pop(field) for field in to_remove]
    tasks = [Task(**data) for data in data_dict]
    raw = Raw(depot, tasks)
    return raw

DATA_GETTER ={
     'csv': _get_data_from_csv
}

def get_data(file_name, file_type = 'csv', depot_as_task = False):
    """
    
    """
    data_path = f'{cfg.DATA_PATH}{file_name}.{file_type}'
    raw = DATA_GETTER[file_type](data_path, depot_as_task)
    return raw


def gen_tasks(num_tasks, max_robots, coor_range = 500, no_delivery = False):
    """
    If no-delivery = true then every tasks has their pick up node = their destination node.
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
        process_time = np.linalg.norm(pickup_coor- dest_coor)
        cur_tasks = Task(pickup_coor, dest_coor, process_time, total_robots_required, i)
        tasks.append(cur_tasks)
    return tasks

def gen_depot(coor_range = 500, as_task = False):
    coor = rnd.uniform(0, coor_range,2)
    if not as_task:
        depot = Node(coor, 0, 0)
    else:
        depot = Task(coor,coor,0,0,0)
    return depot


def gen_data(total_robots,total_tasks, no_delivery = False,as_task = False):
        depot = gen_depot(as_task = as_task)
        tasks = gen_tasks(total_tasks, total_robots, no_delivery = no_delivery)
        raw = Raw(depot, tasks)
        return raw