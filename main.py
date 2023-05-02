#import cac thu cac thu

import config as cfg
from data_acquirer import get_data, gen_data
from cplex_solver import solve

if __name__ == "__main__":

    random_data = True
    total_robots = 5
    data_name = '.'

    if random_data:
        data = gen_data("random",
                        total_robots = 5,
                        total_tasks = 40,
                        no_delivery =True)
    else:
        data = get_data(data_name,
                        total_robots = 5)
    

    solve_result = solve(data,
                         formulation = "vrp_makespan")
    
    