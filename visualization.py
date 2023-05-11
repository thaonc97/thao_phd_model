import copy
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from operator import itemgetter

import config as cfg



def plot_map(raw):
    depot, tasks = raw.unpack()
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

def get_moved_edges(solve_result, correct_sink = False):
    sol = solve_result['sol']
    if sol is None:
        print("Didn't found solution")
        raise
     
    x = solve_result['x']
    sol_dict = sol.get_value_dict(x)
    moved_edges = [edge for edge in sol_dict if sol_dict[edge] ==1]
    if correct_sink:
        moved_edges = _correct_moved_edges(moved_edges)
    return moved_edges

def get_moved_edges_flow(solve_result,correct_sink = True):
    sol = solve_result['sol']
    if sol is None:
        print("Didn't found solution")
        raise   
    f = solve_result['f']
    moved_edges = []
    for edge in f:
        for _ in range(int(f[edge])):
            moved_edges.append(edge)
    
    if correct_sink == True:
        moved_edges = _correct_moved_edges(moved_edges)
    return moved_edges

def _correct_moved_edges(moved_edges,sink_last = True):
    sink_idx = max(moved_edges,key=itemgetter(1))[1]
    corrected_moved_edges = []
    for edge in moved_edges:
        if edge[1] == sink_idx:
            cur_edge = list(edge)
            cur_edge[1] = 0
            corrected_moved_edges.append(tuple(cur_edge))
        else:
            corrected_moved_edges.append(edge)
    
    return corrected_moved_edges

def get_paths(moved_edges,total_robots):
    """
    Acquire the node sequence of each robot
    """
    left_edges = copy.deepcopy(moved_edges)
    paths = []
    for _ in range(total_robots):
        cur_path =[0]
        for edge in left_edges:
            if edge[0] ==0:
                cur_node = edge[1]
                cur_path.append(edge[1])
                left_edges.remove(edge)
                break

        while cur_node !=0:
            for edge in left_edges:
                if edge[0] == cur_node:
                    cur_node = edge[1]
                    cur_path.append(edge[1])
        paths.append(cur_path)
    
    return paths

def get_tasks_path(paths,nodes):
    """
    Acquire the task sequence for each robot
    """
    node_idx_mapper = {node.index: node for node in nodes}
    tasks_path = []
    for path in paths:
        task_path = [node_idx_mapper[node].task_idx for node in path]
        tasks_path.append(task_path)
    return tasks_path

def get_paths_coor(paths,node_idx_mapper):
    paths_coor = []
    for path in paths:
        xs = [node_idx_mapper[node].coor[0] for node in path]
        ys = [node_idx_mapper[node].coor[1] for node in path]
        cur_path_coors = [xs,ys]
        paths_coor.append(cur_path_coors)
    
    return paths_coor

def get_edge_paths(paths):
    edge_paths = []
    for path in paths:
        cur_edge_path = []
        for i in range(len(path)-1):
            cur_edge_path.append((path[i], path[i+1]))
        edge_paths.append(cur_edge_path)
    return edge_paths