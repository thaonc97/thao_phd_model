#import cac thu cac thu

import config as cfg
from data_acquirer import get_data, gen_data
from solver import solve_pdpcr

if __name__ == "__main__":

    random_data = True
    total_robots = 5
    total_tasks = 40
    data_name = '.'

    if random_data:
        data = gen_data(total_robots, total_tasks, False)
    else:
        data = get_data(data_name, total_robots)
    

    solve_result = solve_pdpcr(data, total_robots,1)
    