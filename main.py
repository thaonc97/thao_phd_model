
import config as cfg
from data_acquirer import get_data, gen_data
from data_processor import process
from solver import solve_pdpcr
import visualization
import helpers
import heuristic

import copy

import numpy as np



if __name__ == "__main__":

    """
    SOLVE_METHOD ={
    1: solve_vrp_sumofcosts,
    2: solve_flow_makespan,
    3: solve_vrp_makespan,
    4: solve_vrp_ver2,
    5: solve_vrp_ver2_makespan,
    }
    """

    method = 5

    total_robots = 5
    random_data = False

    data_type = 'wr2'
    # data_name = 'generated3.csv'
    data_name = 'wr2/bar-n100-1-3.csv' #real data
    dist_path = 'wr2/bar-n100-1-dist.csv' #real data
    total_tasks = 6 # random data
    
    if False:
        is_depot_task = True
    else:
        is_depot_task = False #TODO remove this
    if random_data:
        raw = gen_data(total_robots, total_tasks, False, is_depot_task)
    else:
        raw = get_data(data_name, data_type, is_depot_task)

    print(f'Solving for {total_robots} robots and {len(raw.tasks)} tasks')
    processed_data = process(raw, method) 
    solve_result = solve_pdpcr(processed_data, total_robots, method,dist_path)

    # processed_h = heuristic.process_greedy_1(raw)
    # solve_result_h = heuristic.solve_greedy_1(processed_h, total_robots)
    
    # node_idx_mapper = {node.index: node for node in processed_data.nodes}
    # moved_edges = visualization.get_moved_edges(solve_result,False)
    # paths = visualization.get_paths(moved_edges,total_robots)
    # tasks_path = visualization.get_tasks_path(paths, processed_data.nodes)
    # paths_coor = visualization.get_paths_coor(paths,node_idx_mapper)
    # edge_paths = visualization.get_edge_paths(paths)
    print('Done')