import copy
import numpy as np

import helpers
from problem_variables import Raw, Task, Processed

class Path:
    def __init__(self,depot):
        self.tasks =[depot]
        self.total_time = 0
        self.travel_time = 0
        pass
    
    def insert(self, task):
        self.tasks.append(task)
        self.update_travel_time(task)
        return
    
    def update_total_time(self, time):
        self.total_time = time
        return
    
    def update_travel_time(self,task):
        last_task = self.get_last_task()
        self.travel_time += helpers.cal_dist_task(last_task, task)
#         return travel_time
    
    def add_total_time(self,time_to_add):
        self.total_time += time_to_add
        return
    
    def get_insert_cost(self, task, c_travel, c_operate):
        last_task = self.get_last_task()
        cost_if_insert = self.total_time + c_travel[last_task.index][task.index] + c_operate[task.index]
        return cost_if_insert
    
    def get_last_task(self):
        return self.tasks[-1]
    
    def __repr__(self):
            return f"Path {[task.index for task in self.tasks]}"
    

class Solution:
    def __init__(self, paths, depot, tasks):
        self.paths = paths
        self.depot = depot
        self.tasks = tasks
    
    def get_makespan(self):
        makespan = max([path.total_time for path in self.paths])
        return makespan
    
    def __repr__(self):
        return f"Paths {self.paths}"


def process_greedy_1(raw: Raw):
    raw_depot = raw.depot
    depot_coor = raw_depot.coor
    depot = Task(depot_coor,depot_coor,0,0,0)
    tasks = raw.tasks
    processed_data = Processed(depot, tasks, None, None)
    return processed_data 

def solve_greedy_1(processed:Processed, total_robots): # 
    depot = processed.depot
    tasks = processed.tasks

    c_travel = helpers.create_task_dist_dict(depot,tasks)
    c_operate = helpers.create_operate_dict(depot,tasks)
    c_travel_back  ={}
    for i in range(len(c_travel)):
        c_travel[i].pop(i)
        try:
            c_travel_back[i] = c_travel[i].pop(0)
        except:
            pass


    uninserted = copy.deepcopy(tasks) #j left
    uninserted_idx = [task.index for task in uninserted]
    paths = [Path(depot) for _ in range(total_robots)]
    cur_sol = Solution(paths, depot, tasks)
    best_obj = 0

    idx_task_mapper = {task.index: task for task in tasks}

    C_max = 0
    while len(uninserted_idx) > 0:
        shortest_path_length = 9_999_999
        for path in cur_sol.paths:
            last_task = path.get_last_task()
            last_path_idx = last_task.index
            closest_idx = min(c_travel[last_path_idx],key = c_travel[last_path_idx].get)
            closest_task = idx_task_mapper[closest_idx]
            cost_if_insert = path.get_insert_cost(closest_task,c_travel,c_operate)
        
            if cost_if_insert < shortest_path_length:
                shortest_path_length = cost_if_insert
                best_path = path
                best_task = idx_task_mapper[closest_idx]
        
        # print(f"CURRENT BEST PATH: {best_path} \n BEST TASK: {best_task}")
        # Find remaining paths and insert the select task to them
        cost_if_insert_dict = {path: path.get_insert_cost(best_task,c_travel, c_operate) for path in paths}
        sorted_cost = list(np.sort(list(cost_if_insert_dict.values())))
        shortest_to_insert_costs = sorted_cost[:best_task.n]
        inserted_paths_length = shortest_to_insert_costs[-1]

        for _ in range(best_task.n):
            cur_length = shortest_to_insert_costs.pop(0)
            for path, length in cost_if_insert_dict.items():
                if length == cur_length:
                    path.insert(best_task)
                    path.update_total_time(inserted_paths_length)
                    cost_if_insert_dict.pop(path)
                    break
            
                    
        for i in range(len(c_travel)):
            try:
                c_travel[i].pop(best_task.index)
            except:
                pass
        
        uninserted_idx.remove(best_task.index)
    
    # insert 0 to path
    for path in paths:
        last_idx = path.get_last_task().index
        path.add_total_time(c_travel_back[last_idx])
        path.insert(depot)
        
        
    print(cur_sol.paths)
    print(f"The heuristic solution is {cur_sol.get_makespan()}")
    return cur_sol 

def solve_greedy_jp(processed:Processed, total_robots): # 
    depot = processed.depot
    tasks = processed.tasks

    c_travel = helpers.create_task_dist_dict(depot,tasks)
    c_operate = helpers.create_operate_dict(depot,tasks)
    c_travel_back  ={}
    for i in range(len(c_travel)):
        c_travel[i].pop(i)
        try:
            c_travel_back[i] = c_travel[i].pop(0)
        except:
            pass


    uninserted = copy.deepcopy(tasks) #j left
    uninserted_idx = [task.index for task in uninserted]
    paths = [Path(depot) for _ in range(total_robots)]
    cur_sol = Solution(paths, depot, tasks)
    best_obj = 0

    idx_task_mapper = {task.index: task for task in tasks}

    C_max = 0
    while len(uninserted_idx) > 0:
        shortest_path_length = 9_999_999
        for path in cur_sol.paths:
            last_task = path.get_last_task()
            last_path_idx = last_task.index
            closest_idx = min(c_travel[last_path_idx],key = c_travel[last_path_idx].get)
            closest_task = idx_task_mapper[closest_idx]
            cost_if_insert = path.get_insert_cost(closest_task,c_travel,c_operate)
        
            if cost_if_insert < shortest_path_length:
                shortest_path_length = cost_if_insert
                best_path = path
                best_task = idx_task_mapper[closest_idx]
        
        # print(f"CURRENT BEST PATH: {best_path} \n BEST TASK: {best_task}")
        # Find remaining paths and insert the select task to them
        cost_if_insert_dict = {path: path.get_insert_cost(best_task,c_travel, c_operate) for path in paths}
        sorted_cost = list(np.sort(list(cost_if_insert_dict.values())))
        shortest_to_insert_costs = sorted_cost[:best_task.n]
        inserted_paths_length = shortest_to_insert_costs[-1]

        for _ in range(best_task.n):
            cur_length = shortest_to_insert_costs.pop(0)
            for path, length in cost_if_insert_dict.items():
                if length == cur_length:
                    path.insert(best_task)
                    path.update_total_time(inserted_paths_length)
                    cost_if_insert_dict.pop(path)
                    break
            
                    
        for i in range(len(c_travel)):
            try:
                c_travel[i].pop(best_task.index)
            except:
                pass
        
        uninserted_idx.remove(best_task.index)
    
    # insert 0 to path
    for path in paths:
        last_idx = path.get_last_task().index
        path.add_total_time(c_travel_back[last_idx])
        path.insert(depot)
        
        
    print(cur_sol.paths)
    print(f"The heuristic solution is {cur_sol.get_makespan()}")
    return cur_sol 