import copy
import csv
from docplex.mp.model import Model
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import pandas as pd
import time
import config as cfg

from problem_variables import Task, Node

seed_num =  cfg.RND_SEED
rnd = np.random
rnd.seed(seed_num)


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