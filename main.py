
import config as cfg
from data_acquirer import get_data, gen_data
from data_processor import process
from solver import solve_pdpcr
import visualization
import helpers
import heuristic

import copy
import logging
import numpy as np



if __name__ == "__main__":
    logging.basicConfig(format=cfg.LOG_FORMATTER, level=cfg.LOG_LEVEL, datefmt='%d-%b-%y %H:%M:%S')
    """
    SOLVE_METHOD ={
    1: solve_vrp_sumofcosts,
    2: solve_flow_makespan,
    3: solve_vrp_makespan,
    4: solve_vrp_ver2,
    5: solve_vrp_ver2_makespan,
    6: solve_flow_ver2_makespan
    }
    """

    method = 2 #exact method

    total_robots = 10
    random_data = False

    data_type = 'csv'
    # data_name = 'data2.csv'
    # data_name = 'generated18_large.csv'  #Check this
    data_name = 'lim/200/LC1_2_1-5-uniform.csv'
    # dist_path = 'wr2/bar-n100-1-dist.csv' #real data
    total_tasks = 6 # random data
    

    is_depot_task = False 
    if random_data:
        raw = gen_data(total_robots, total_tasks, False, is_depot_task)
    else:
        raw = get_data(data_name, data_type, is_depot_task)

    print(f'Solving for {total_robots} robots and {len(raw.tasks)} tasks')
    #Solve with CPLEX
    # processed_data = process(raw, method)
    # solve_result = solve_pdpcr(processed_data, total_robots, method)
    # solve_result['sol'].display()

    #Solve with greedy JP
    processed_h = heuristic.process_greedy_1(raw)
    solve_result_h_jp = heuristic.solve_greedy_jp_compare_start_time(processed_h, total_robots)
    solve_result_h_jp.get_makespan()
    print(solve_result_h_jp.paths)

    #Solve with 2-opt
    depot = processed_h.depot
    tasks = processed_h.tasks
    c_travel = helpers.create_task_dist_dict(depot,tasks)
    c_operate = helpers.create_operate_dict(depot,tasks)
    # solve_opt = heuristic.TwoOptSolver(processed_h,c_travel, c_operate)
    # solve_opt.solve(total_robots= total_robots)

    #Solve with Viet generalized Solver
    # task_each_ite = 'random'
    # solve_viet = heuristic.VGeneralizedSolver(processed_h,c_travel, c_operate)
    # sol = solve_viet.solve(total_robots,task_each_ite, False, task_range = [1,4])
    # print(sol.paths)
    # print(f' VGeneralized Solver makespan: {sol.get_makespan()}')

    # solve_viet_orig = heuristic.VietSolver(processed_h,c_travel, c_operate)
    # solve_viet_orig.solve(total_robots,task_each_ite, False)
    # print(f'Paths Viet Solver {solve_viet_orig.paths}')
    # print(f' Viet Solver makespan: {solve_viet_orig.get_makespan()}')
    # print('Done')

    #Solve with Tabu search
    tabu = heuristic.TabuSearch(processed_h,c_travel, c_operate)
    tabu.search(total_robots,cfg.TABU_MAX_ITER)


def mass_solve_wr2(instance_paths:list, dist_paths:list, total_robots:int, method :int = 5):
    
    for (instance,path) in zip(instance_paths,dist_paths):
        raw = get_data(instance, 'wr2', False)
        processed_data = process(raw, method) 
        solve_result = solve_pdpcr(processed_data, total_robots, method,path)
        print(f'Solved {instance} with {total_robots} robots')
        print(f'Cost: {solve_result.cost}')
        print(f'Runtime: {solve_result.runtime}')
        print('------------------')
