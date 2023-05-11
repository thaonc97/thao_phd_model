#import cac thu cac thu

import config as cfg
from data_acquirer import get_data, gen_data
from data_processor import process
from solver import solve_pdpcr
import visualization


if __name__ == "__main__":

    """
    SOLVE_METHOD ={
    1: solve_vrp_sumofcosts,
    2: solve_flow_makespan,
    3: solve_vrp_makespan,
    }
    """

    method = 1
    total_robots = 3
    random_data = True

    data_name = 'data4' #real data
    total_tasks = 10 # random data
    

    if random_data:
        raw = gen_data(total_robots, total_tasks, False)
    else:
        raw = get_data(data_name)
    
    processed_data = process(raw, method) 
    solve_result = solve_pdpcr(processed_data, total_robots, method)

    node_idx_mapper = {node.index: node for node in processed_data.nodes}
    moved_edges = visualization.get_moved_edges(solve_result,False)
    paths = visualization.get_paths(moved_edges,total_robots)
    tasks_path = visualization.get_tasks_path(paths, processed_data.nodes)
    paths_coor = visualization.get_paths_coor(paths,node_idx_mapper)
    edge_paths = visualization.get_edge_paths(paths)
    print('Done')