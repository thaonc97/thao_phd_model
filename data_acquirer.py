import numpy as np
import pandas as pd

import config as cfg
from problem_variables import Task, Node


seed_num =  cfg.RND_SEED
rnd = np.random
rnd.seed(seed_num)

def _get_data_from_csv(path):
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

DATA_GETTER ={
     'csv': _get_data_from_csv
}

def get_data(path, file_type = 'csv'):
    """
    
    """
    depot, tasks = DATA_GETTER[file_type](path)
    return depot,tasks


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
        cur_tasks = Task(pickup_coor, dest_coor, total_robots_required, i)
        tasks.append(cur_tasks)
    
    return tasks

def gen_depot(coor_range = 500):
    coor = rnd.uniform(0, coor_range,2)
    depot = Node(coor, 0, 0)
    return depot

def gen_data(total_robots = 5,total_tasks = 40, no_delivery = False):
        depot = gen_depot()
        tasks = gen_tasks(total_tasks, total_robots, no_delivery = no_delivery)
        return depot, tasks