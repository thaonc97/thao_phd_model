
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

    total_robots = 10
    random_data = False

    data_type = 'csv'
    data_name = '/lim/lc107-5-uniform.csv'
    # data_name = 'wr2/bar-n100-1-3.csv' #real data
    # dist_path = 'wr2/bar-n100-1-dist.csv' #real data
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
    # processed_data = process(raw, method)
    # solve_result = solve_pdpcr(processed_data, total_robots, method)

    processed_h = heuristic.process_greedy_1(raw)
    solve_result_h = heuristic.solve_greedy_1(processed_h, total_robots)
    solve_result_h_jp = heuristic.solve_greedy_jp(processed_h, total_robots)
    
    print('Done')


def mass_solve_wr2(instance_paths:list, dist_paths:list, total_robots:int, method :int = 5):
    
    for (instance,path) in zip(instance_paths,dist_paths):
        raw = get_data(instance, 'wr2', False)
        processed_data = process(raw, method) 
        solve_result = solve_pdpcr(processed_data, total_robots, method,path)
        print(f'Solved {instance} with {total_robots} robots')
        print(f'Cost: {solve_result.cost}')
        print(f'Runtime: {solve_result.runtime}')
        print('------------------')
