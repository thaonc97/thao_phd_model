
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

    method = 2

    total_robots = 3
    random_data = True

    data_name = 'data1' #real data
    total_tasks = 20 # random data
    
    if False:
        is_depot_task = True
    else:
        is_depot_task = False #TODO remove this
    if random_data:
        raw = gen_data(total_robots, total_tasks, False, is_depot_task)
    else:
        raw = get_data(data_name,'csv', is_depot_task)

    print(f'Solving for {total_robots} robots and {len(raw.tasks)} tasks')
    processed_data = process(raw, method) 
    solve_result = solve_pdpcr(processed_data, total_robots, method)

    print(f"CPLEX makespan: {solve_result['sol']}")

    processed_h = heuristic.process_greedy_1(raw)
    solve_result_h = heuristic.solve_greedy_1(processed_h, total_robots)
    # node_idx_mapper = {node.index: node for node in processed_data.nodes}
    # moved_edges = visualization.get_moved_edges(solve_result,False)
    # paths = visualization.get_paths(moved_edges,total_robots)
    # tasks_path = visualization.get_tasks_path(paths, processed_data.nodes)
    # paths_coor = visualization.get_paths_coor(paths,node_idx_mapper)
    # edge_paths = visualization.get_edge_paths(paths)
    print('Done')

# if __name__ == "__main__":
#     total_robots = 2
#     total_tasks = 200
#     is_depot_task = True
#     random_data = False
#     data_name = 'data3'

#     if random_data:
#         raw = gen_data(total_robots, total_tasks, False, is_depot_task)
#     else:
#         raw = get_data(data_name,'csv', is_depot_task)

#     c_travel = helpers.create_task_dist_dict(raw.depot,raw.tasks)
#     c_operate = helpers.create_operate_dict(raw.depot,raw.tasks)
#     for i in range(len(c_travel)):
#         c_travel[i].pop(i)
#         try:
#             c_travel[i].pop(0)
#         except:
#             pass


#     uninserted = copy.deepcopy(raw.tasks) #j left
#     uninserted_idx = [task.index for task in uninserted]
#     paths = [heuristic.Path(raw.depot) for _ in range(total_robots)]
#     cur_sol = heuristic.Solution(paths,raw.depot, raw.tasks)
#     best_obj = 0

#     idx_task_mapper = {task.index: task for task in raw.tasks}

#     C_max = 0
#     while len(uninserted_idx) > 0:
#         shortest_path_length = 9_999_999
#         for path in cur_sol.paths:
#             last_task = path.get_last_task()
#             last_path_idx = last_task.index
#             closest_idx = min(c_travel[last_path_idx],key = c_travel[last_path_idx].get)
#             closest_task = idx_task_mapper[closest_idx]
#             cost_if_insert = path.get_insert_cost(closest_task,c_travel,c_operate)
        
#             if cost_if_insert < shortest_path_length:
#                 shortest_path_length = cost_if_insert
#                 best_path = path
#                 best_task = idx_task_mapper[closest_idx]
        
#         print(f"CURRENT BEST PATH: {best_path} \n BEST TASK: {best_task}")
#         # Find remaining paths and insert the select task to them
#         cost_if_insert_dict = {path: path.get_insert_cost(best_task,c_travel, c_operate) for path in paths}
#         sorted_cost = list(np.sort(list(cost_if_insert_dict.values())))
#         shortest_to_insert_costs = sorted_cost[:best_task.n]
#         inserted_paths_length = shortest_to_insert_costs[-1]

#         for _ in range(best_task.n): # Insert
#             cur_length = shortest_to_insert_costs.pop(0)
#             for path, length in cost_if_insert_dict.items():
#                 if length == cur_length:
#                     path.insert(best_task)
#                     path.update_total_time(inserted_paths_length)
#                     cost_if_insert_dict.pop(path)
#                     break
            
                    
#         for i in range(len(c_travel)):
#             try:
#                 c_travel[i].pop(best_task.index)
#             except:
#                 pass
        
#         uninserted_idx.remove(best_task.index)
    
#     print(cur_sol.paths)
#     print(f"The solution is {cur_sol.get_makespan()}")
#         #Remember to remove best_task from c_travel