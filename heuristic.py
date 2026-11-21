import copy
from docplex.mp.model import Model
import heapq
import networkx as nx
import numpy as np
import pandas as pd
import pickle
import random 
import time 
import os

import config as cfg
from collections import deque
import helpers
import logging
from problem_variables import Raw, Task, Processed

"""
In some algorithms which differientiate between the source and the sink, then
the source index is 0, sink's is -1
"""
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
    
    def update_travel_time(self, task):
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

    def get_start_insert_cost(self, task, c_travel):
        """Get start cost of inserting a task to a path

        Parameters
        ----------
        task : _type_
            _description_
        c_travel : _type_
            _description_

        Returns
        -------
        _type_
            _description_
        """ 
        last_task = self.get_last_task()
        cost_if_insert = self.total_time + c_travel[last_task.index][task.index]
        return cost_if_insert

    def get_last_task(self):
        return self.tasks[-1]
    
    def __repr__(self):
            return f"Path {[task.index for task in self.tasks]}"
    
    def __getitem__(self, i):
        return self.tasks[i]

class Solution:
    def __init__(self, paths, depot, tasks):
        self.paths = paths
        self.depot = depot
        self.tasks = tasks
        self.starting_times = {}
    
    def get_makespan(self):
        makespan = max([path.total_time for path in self.paths])
        return makespan
    
    def __repr__(self):
        return f"Paths {self.paths}"
    
    def get_idx_paths(self,change_sink_idx = False):
        idx_paths = []
        for path in self.paths:
            idx_paths.append([task.index for task in path.tasks])
            if change_sink_idx:
                idx_paths[-1][-1] = -1
        return idx_paths


def process_greedy_1(raw: Raw):
    raw_depot = raw.depot
    depot_coor = raw_depot.coor
    depot = Task(depot_coor,depot_coor,0,0,0)
    tasks = raw.tasks
    processed_data = Processed(depot, tasks, None, None)
    return processed_data 

def solve_greedy_1(processed:Processed, total_robots): 
    """Deprecated. Use solve_greedy_jp and other variants instead

    Parameters
    ----------
    processed : Processed
        
    total_robots : int
        number of robots
    """
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
        
        
    # for path in cur_sol.paths:
    #     print(path)
    print(f"Thao heuristics sol {cur_sol.get_makespan().round(2)}")
    return cur_sol

def solve_greedy_jp(processed:Processed, total_robots):
    "Do compare process time"
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
    uninserted_idx = [task.index for task in tasks] #J_t
    paths = [Path(depot) for _ in range(total_robots)]
    cur_sol = Solution(paths, depot, tasks)
    C_max = 0 

    idx_task_mapper = {task.index: task for task in tasks}

    C_max = 0
    while len(uninserted) > 0:
        c_max = np.infty
        for task in uninserted:
            task_index = task.index
            L = {}
            for path in cur_sol.paths:
                cost_if_insert = path.get_insert_cost(task,c_travel, c_operate)
                L[path] = cost_if_insert
            df_L = pd.DataFrame(list(L.items()),columns = ['paths','cost_if_insert']).sort_values('cost_if_insert')
            n = task.n
            n_th_smallest_cost = df_L.iloc[n-1]['cost_if_insert']
            if n_th_smallest_cost < c_max:
                c_max = n_th_smallest_cost
                best_task = task
                best_paths = df_L.iloc[:n]['paths'].tolist()
                
        cur_sol.starting_times[best_task] = c_max
        for path in best_paths:
            path.insert(best_task)
            path.update_total_time(c_max)
        C_max = max(C_max,c_max)
        uninserted.remove(best_task)

    # insert 0 to path
    for path in paths:
        last_idx = path.get_last_task().index
        path.add_total_time(c_travel_back[last_idx])
        path.insert(depot)

    # for path in cur_sol.paths:
    #     print(path)
    print(f'JP heuristics sol is {cur_sol.get_makespan().round(2)}')

    return cur_sol

def solve_greedy_jp_compare_start_time(processed:Processed, total_robots):
    #TODO
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
    # uninserted_idx = [task.index for task in tasks] #J_t
    paths = [Path(depot) for _ in range(total_robots)]
    cur_sol = Solution(paths, depot, tasks)
    C_max = 0 

    idx_task_mapper = {task.index: task for task in tasks}

    C_max = 0
    while len(uninserted) > 0:
        c_max = np.infty
        for task in uninserted:
            task_index = task.index
            L = {}
            for path in cur_sol.paths:
                start_cost_if_insert = path.get_start_insert_cost(task, c_travel)
                L[path] = start_cost_if_insert
            df_L = pd.DataFrame(list(L.items()),columns = ['paths','cost_if_insert']).sort_values('cost_if_insert')
            # get the n th smallest cost_if_insert
            # n = idx_task_mapper[task_index].n
            n = task.n
            n_th_smallest_cost = df_L.iloc[n-1]['cost_if_insert']
            if n_th_smallest_cost < c_max:
                c_max = n_th_smallest_cost
                best_task = task
                best_paths = df_L.iloc[:n]['paths'].tolist()
                
        c_max += c_operate[best_task.index]
        for path in best_paths:
            path.insert(best_task)
            path.update_total_time(c_max)
        # print(f'\n path length {[(path,path.total_time) for path in paths]}')
        C_max = max(C_max,c_max)
        cur_sol.starting_times[best_task] = c_max
        uninserted.remove(best_task)

    # insert 0 to path
    for path in paths:
        last_idx = path.get_last_task().index
        path.add_total_time(c_travel_back[last_idx])
        path.insert(depot)

    # for path in cur_sol.paths:
        # print(path)
    print(f'JP heuristics sol is {cur_sol.get_makespan().round(2)}')

    return cur_sol


class TwoOptSolver:
    """Solve PDPCR using 2-opt heuristic
    We need a function to initialize the solution first.
    Then build a graph from the set of paths, need a dict to save a set of 2-opt-available nodes for each node.
    And we only apply 2opt on the set of 2-opt-available nodes, instead of check for apply-able everytime.
    To recalculate the cost
    """
    def __init__(self, processed, c_travel, c_operate, *kwargs):
        self.processed = processed
        self.c_travel = c_travel
        self.c_operate = c_operate
        if 'init_function' in kwargs:
            self.init_function = kwargs['init_function']

    def _to_graph(self, paths: list):
        """
        Return a networkx graph from the set of paths
        """
        G = nx.MultiDiGraph()
        edges = self._paths_to_edges(paths)
        G.add_edges_from(edges)
        return G


    def _paths_to_edges(self,paths: list): 
        """
        extract the edges to the directed graph G from a
        set of paths. More on section 2 in "Formal construct
        solution from paths 13-12-2023" document
        
        Parameters
        ----------
        paths : list
            list of tuples, every tuple is a path of every robot
            excluding the sink and the source (nessessary?)
            
        Returns
        -------
        edges : list
            list of edges, note that there may be duplications edges
            
        """
        edges = []
        for path in paths:
            cur_edges = self._path_to_edges(path)
            edges += cur_edges
        return edges


    def _path_to_edges(self, path : list):
        edges = []
        cur_len = len(path)
        if path[0] != 0:
            edges.append((0,path[0]))
        for i in range(cur_len - 1):
            edges.append((path[i],path[i + 1]))
        # edges.append((path[-1],-1))
        return edges
    
    def _apply_2opt(self, G ,edge1, edge2):
        """
        Apply 2-opt on the graph G
        """
        (a,b) = edge1
        (c,d) = edge2
        
        G.remove_edge(a,b)
        G.remove_edge(c,d)
        G.add_edge(a,d)
        G.add_edge(c,b)
        return G
    
    def _two_opt_able(self, edge1, edge2, descendants):
        """
        Check if 2 edges can be applied 2-opt
        """
        (a,b) = edge1
        (c,d) = edge2
        if a==d or a==c or c==b or c==d:
            return False
        elif a in descendants[d]:
            return False
        elif c in descendants[b]:
            return False
        else:
            return True

    def _get_out_nodes(self,graph):
        """
        Get a dictionary of out node from all nodes on a directed graph
        
        Parameters
        ----------
        graph : nx.DiGraph or nx.MultiDiGraph


        Returns
        -------
        out_nodes : dict
            `outnodes[1] = [2,3,69]` means that edges (1,2), (1,3), (1,69) in the graph.
        """
        out_nodes = {}
        nodes = list(graph.nodes())
        for node in nodes:
            out_nodes[node] = list(graph.successors(node))

        return out_nodes

    def _get_start_time(self, graph, c_travel, c_operate):
        """Get the start time of each task in the graph

        Parameters
        ----------
        graph : _type_
            _description_
        c_travel : _type_
            _description_
        c_operate : _type_
            _description_

        Returns
        -------
        _type_
            _description_
        """
        reversed_topo_order = list(reversed(list(nx.topological_sort(graph))))
        out_nodes = self._get_out_nodes(graph)

        c_operate[-1] = 0
        c_travel[-1] = {}
        c_travel[0][-1] = 0
        for i in c_travel:
            if i!= 0 and i!=-1:
                c_travel[i][-1] = c_travel[i][0]
        
        z = {}
        for j in c_operate:
            z[j] = -np.infty
        z[0] = 0
        z[-1] = -np.infty
        
        while reversed_topo_order:
            cur_task = reversed_topo_order.pop()
            for succ in out_nodes[cur_task]:
                z_cur = z[cur_task] + c_operate[cur_task] + c_travel[cur_task][succ]
                if z[succ] < z_cur:
                    z[succ] = z_cur
        return z

    def _change_sink_idx(self, paths):
        """Change the idx of the depot at the end of each path to -1

        Parameters
        ----------
        paths : list
            list of path of each solution
        """
        for path in paths:
            path[-1] = -1
            return paths

    def solve(self, total_robots, init_function= solve_greedy_jp_compare_start_time):
        """
        Solve PDPCR using 2-opt heuristic
        """
        init_sol = init_function(self.processed, total_robots)
        cur_C_max = init_sol.get_makespan()
        print(f'Initial makespan: {cur_C_max}')
        idx_paths = init_sol.get_idx_paths(change_sink_idx = True)
        G = self._to_graph(idx_paths)

        finish_loop = False
        restart_loop = False
        while finish_loop == False:
            restart_loop= False
            descendants = {node: nx.descendants(G,node) for node in G.nodes}
            edges = list(G.edges())
            # get 2 different elements from edges:
            for i in range(len(edges)):
                if restart_loop == True:
                    break
                for j in range(i+1,len(edges)):
                    edge1 = edges[i]
                    edge2 = edges[j]
                    if self._two_opt_able(edge1, edge2, descendants) == True: 
                        G_p = copy.deepcopy(G)
                        G_p = self._apply_2opt(G_p, edge1, edge2)
                        new_C_max = self._get_start_time(G_p, self.c_travel, self.c_operate)[-1]
                        if new_C_max < cur_C_max:
                            cur_C_max = new_C_max
                            print(f"Applied 2-opt successfully on edges {edge1} and {edge2}, new makespan: {cur_C_max}")
                            G = G_p
                            restart_loop = True
                            break
            if restart_loop == False:
                finish_loop = True
        return G, cur_C_max


class VietSolver:
    """Solve PDPCR applying Viet's idea (assign k task every iteration)
    using CPLEX
    """
    def __init__(self, processed, c_travel, c_operate, *kwargs):
        self.processed = processed
        self.depot = processed.depot
        self.tasks = processed.tasks
        self.c_travel = c_travel
        self.c_operate = c_operate
        self.tasks_remains = [task for task in processed.tasks]
        self.idx_task_mapper = {task.index: task for task in processed.tasks}

    def get_makespan(self):
        makespan = max([path.total_time for _,path in self.paths.items()])
        return makespan

    def _all_tasks_finished(self):
        all_finished = False
        if len(self.tasks_remains) == 0:
            all_finished = True
        return all_finished
    
    def _cal_Jk(self,k):
        """Return total maximum number of robots might be needed
        to assign to k tasks

        Parameters
        ----------
        k : int
            number of tasks to be assigned

        Returns
        -------
        J_k : int
        """
        ns = [task.n for task in self.tasks_remains]
        k_largest = heapq.nlargest(k,ns)
        J_k = sum(k_largest)
        return J_k

    def _solve_viet(self, 
                    total_robots, 
                    tasks_each_iteration, 
                    consider_process_time):
        
        starting_times = {}
        for robot,value in self.paths.items():
            starting_times[robot] = value.total_time
        J_k = self._cal_Jk(tasks_each_iteration)
        if J_k <= total_robots:
            print(f'J_k = {J_k} <= {total_robots} = total_robots, solving first case')
            self._lastest_solve_result = self._solve_first_case(total_robots, 
                                                                starting_times, 
                                                                tasks_each_iteration, 
                                                                consider_process_time)
            self.lastest_case = 1
        else: 
            print(f'J_k = {J_k} > total_robots = {total_robots}, solving second case')
            self._lastest_solve_result = self._solve_second_case(total_robots, 
                                                                 starting_times, 
                                                                 tasks_each_iteration, 
                                                                 consider_process_time)
            self.lastest_case = 2
        #TODO
    
    def _update_paths(self):
        """Update the paths of each robot as well as procesed time
        """
        max_times = {task.index: 0 for task in self.tasks_remains}
        affected_task_idcs = []
        x_results = self._lastest_solve_result['sol'].get_value_dict(self._lastest_solve_result['x'], keep_zeros = False)
        if self.lastest_case == 2:
            v_results_one = self._lastest_solve_result['sol'].get_value_dict(self._lastest_solve_result['v'],keep_zeros = False)

        for key, value in x_results.items():
            if helpers.h_equal(value,1):
                robot_idx = key[0]
                task_idx = key[1]
                if self.lastest_case ==2 and task_idx in v_results_one:
                    continue
                try:
                    affected_task_idcs.append(task_idx)
                    cur_path_last_idx = self.paths[robot_idx][-1].index
                    cur_time = self.paths[robot_idx].total_time + self.c_travel[cur_path_last_idx][task_idx] 
                    max_times[task_idx] = max(max_times[task_idx], cur_time)
                    self.paths[robot_idx].insert(self.idx_task_mapper[task_idx])
                except Exception as e:
                    print(e)
                    pass
        
        affected_task_idcs = set(affected_task_idcs)
        for path_idx,path in self.paths.items():
            last_task_idcs = path[-1].index
            if last_task_idcs in affected_task_idcs:
                time_to_update =max_times[last_task_idcs] +self.c_operate[last_task_idcs]
                path.update_total_time(time_to_update)
        
        return

    
    def _remove_processed_tasks(self):
        if self.lastest_case == 2:
            solved_by_artificial_robot = self._lastest_solve_result['sol'].get_value_dict(self._lastest_solve_result['v'],keep_zeros = False)
        for key, value in self._lastest_solve_result['sol'].get_value_dict(self._lastest_solve_result['x'],keep_zeros = False).items():
            task_idx = key[1]
            if self.lastest_case == 2:
                if task_idx in solved_by_artificial_robot:
                    continue
            procesed_task = self.idx_task_mapper[task_idx]
            #remove processed_task from self.tasks_remains
            try:
                self.tasks_remains.remove(procesed_task)
            except:
                pass
    
    def _robot_returns(self):
        for path_idx,path in self.paths.items():
            last_task_idx = path[-1].index
            path.add_total_time(self.c_travel[last_task_idx][0])
            path.insert(self.depot)
        
        return

    def solve(self, total_robots, tasks_each_iteration, consider_process_time):
        self.paths ={i:Path(self.depot) for i in range(total_robots)}
        while self._all_tasks_finished() == False:
            logging.info(f'Start solving')
            self._solve_viet(total_robots, tasks_each_iteration, consider_process_time)
            # logging.info(f'Solved!\n updating paths...')
            self._update_paths()
            # logging.info(f'Done\n Removing processed tasks...')
            self._remove_processed_tasks()
        self._robot_returns()
        #TODO
    
    def _construct_a(self, total_robots):
        """Construct a dictionary of cost from robot i to task j,

        Parameters
        ----------
        total_robots : int
            Total number of robots

        Returns
        -------
        a : dict
            a[i][j] is the cost to go from the current position of robot i
            (the destination of the last task that robot i processed) to
            pick up location of task j
        """
        if not hasattr(self,'paths'):
            self.paths ={i:Path(self.depot) for i in range(total_robots)}
        a = {}
        for i in range(total_robots):
            a[i] = self.c_travel[self.paths[i][-1].index]
        return a

    def _construct_a_artificial(self, total_robots,total_robots_required):
        a = {}
        a = self._construct_a(total_robots)
        for i in range(total_robots, total_robots_required):
            a[i] = {task.index:cfg.BIG_M1 for task in self.tasks_remains}
        return a

    def _construct_t_artificial(self,starting_times,total_robots,total_robots_required):
        for i in range(total_robots, total_robots_required):
            starting_times[i] = 0
        return starting_times

    def _solve_second_case(self,
                           total_robots,
                           starting_times :dict,
                           tasks_each_iteration : int,
                           consider_process_time : bool):
        
        # total_robots_required = np.sum([task.n for task in self.tasks_remains]) # This is probably for assign all 
                                                                                # robots at each iteration case
        J_k = self._cal_Jk(tasks_each_iteration) 
        A_idcs = [task.index for task in self.tasks_remains]
        B_idcs = [j for j in range(total_robots)]
        Bf_idcs = [j for j in range(total_robots, J_k)] #Artificial robots
        Bp_idcs = B_idcs + Bf_idcs
        BpA_idcs = [(i,j) for i in Bp_idcs for j in A_idcs]

        k = tasks_each_iteration
        a = self._construct_a_artificial(total_robots, J_k)
        t = self._construct_t_artificial(starting_times, total_robots, J_k)
        c = self.c_operate
        n ={task.index: task.n for task in self.tasks_remains}

        m = Model('Viet_case_2')
        C_max = m.continuous_var(name = 'C_max')
        x = m.binary_var_dict(BpA_idcs, name = 'x')
        # x = m.continuous_var_dict(BpA_idcs, ub =1, name = 'x')
        p = m.continuous_var_dict(BpA_idcs, name = 'p')
        v = m.binary_var_dict(A_idcs, name = 'v')
        q = m.binary_var_dict(A_idcs, name = 'q')
        d = m.continuous_var(name = 'd')

        m.minimize(C_max +m.sum(cfg.BIG_M1*p[i,j] for j in A_idcs for i in B_idcs))
        if consider_process_time:
            m.add_constraints(C_max >= (a[i][j] + t[i] + c[j])*x[i,j] for i in B_idcs for j in A_idcs)
        else:
            m.add_constraints(C_max >= (a[i][j] + t[i])*x[i,j] for i in B_idcs for j in A_idcs)

        m.add_constraint(m.sum(x[i,j] for i in B_idcs for j in A_idcs) >= 1)
        m.add_constraints(m.sum(x[i,j] for i in Bp_idcs) == n[j]*q[j] for j in A_idcs) 
        m.add_constraints(m.sum(x[i,j] for j in A_idcs) <= 1 for i in Bp_idcs)
        m.add_constraints(x[i,j] <= 1 for i in Bp_idcs for j in A_idcs) # is this nessessary?
        m.add_constraint(m.sum((m.sum(x[i,j] for j in A_idcs) for i in Bp_idcs)) == d)
        m.add_constraints(x[i,j] <= v[j] for i in Bf_idcs for j in A_idcs) # Relation x and v
        m.add_constraints(x[i,j] <= q[j] for i in Bp_idcs for j in A_idcs) # Relation x and q
        m.add_constraint(m.sum(q[j] for j in A_idcs) == min(k,len(self.tasks_remains))) # k tasks done by all robots
        m.add_constraint(m.sum(q[j]*n[j] for j in A_idcs) == d)

        #Constraints regarding p
        m.add_constraints(p[i,j] <= x[i,j] for i in Bp_idcs for j in A_idcs)
        m.add_constraints(p[i,j] <= v[j] for i in Bp_idcs for j in A_idcs)
        m.add_constraints(p[i,j] >= x[i,j] + v[j] -1 for i in Bp_idcs for j in A_idcs)

        m.parameters.mip.tolerances.mipgap = cfg.SOLVE_GAP
        m.time_limit = cfg.SOLVE_TIME
        if cfg.EXPORT_MODEL == True:
            print(m.export_as_lp())
        sol = m.solve(log_output = cfg.LOG_OUTPUT)
        # print(sol.display())
        solved_gap = m.solve_details.mip_relative_gap
        processed_nodes = m.solve_details.nb_nodes_processed
        solve_time = m.solve_details.time
        solve_info = {
            'sol':sol,
            'x':x,
            'v':v,
            'processed_nodes': processed_nodes,
            'gap':solved_gap,
            'solve_time':solve_time
        }
        
        return solve_info

    def _solve_first_case(self, 
                          total_robots, 
                          starting_times :dict, 
                          tasks_each_iteration : int, 
                          consider_process_time : bool):
        A_idcs = [task.index for task in self.tasks_remains]
        B_idcs = [j for j in range(total_robots)]
        BA_idcs = [(i,j) for i in B_idcs for j in A_idcs]

        k = tasks_each_iteration
        a = self._construct_a(total_robots)
        t = starting_times
        c = self.c_operate
        n ={task.index: task.n for task in self.tasks_remains}

        m = Model('Viet_case_1')

        C_MAX = m.continuous_var(name = 'C_MAX')
        C_max = m.continuous_var_dict(A_idcs, name = 'C_max')
        x = m.binary_var_dict(BA_idcs,ub=1, name = 'x')
        q = m.binary_var_dict(A_idcs, name = 'q')
        d = m.continuous_var(name = 'd')
        # m.minimize(m.sum(C_max[j] for j in A_idcs))
        m.minimize(C_MAX)
        if consider_process_time:
            m.add_constraints((t[i] + a[i][j] + c[j])*x[i,j]  <= C_max[j] for i in B_idcs for j in A_idcs)
        else:
            m.add_constraints(C_max[j] <= C_MAX for j in A_idcs)
            m.add_constraints((t[i] + a[i][j])*x[i,j] <= C_max[j] for i in B_idcs for j in A_idcs)

        m.add_constraints(m.sum(x[i,j] for i in B_idcs) == n[j]*q[j] for j in A_idcs)
        m.add_constraints(m.sum(x[i,j] for j in A_idcs) <= 1 for i in B_idcs)
        m.add_constraint(m.sum(q[j] for j in A_idcs) == min(k,len(self.tasks_remains))) # k tasks done by all robots
        m.add_constraints(x[i,j] <= q[j] for i in B_idcs for j in A_idcs) # Relation x and q
        m.add_constraint(m.sum((m.sum(x[i,j] for j in A_idcs) for i in B_idcs)) == d)
        m.add_constraint(m.sum(q[j]*n[j] for j in A_idcs) == d)

        m.parameters.mip.tolerances.mipgap = cfg.SOLVE_GAP
        m.time_limit = cfg.SOLVE_TIME
        if cfg.EXPORT_MODEL == True:
            print(m.export_as_lp())
        sol = m.solve(log_output = cfg.LOG_OUTPUT)

        solved_gap = m.solve_details.mip_relative_gap
        processed_nodes = m.solve_details.nb_nodes_processed
        solve_time = m.solve_details.time
        solve_info = {
            'sol':sol,
            'x':x,
            'processed_nodes': processed_nodes,
            'gap':solved_gap,
            'solve_time':solve_time
        }
        
        return solve_info
    

class VietSolverOrig(VietSolver): 
    """For solve Viet's problem: trying to assign every robots
    at every iterations
    """

    def _cal_RA(self):

        tasks = self.tasks_remains
        R_A = sum([task.n for task in tasks])
        return R_A
    
    def solve(self, total_robots, consider_process_time):
        self.paths ={i:Path(self.depot) for i in range(total_robots)}
        while self._all_tasks_finished() == False:
            logging.info(f'Start solving')
            self._solve_viet(total_robots, consider_process_time)
            logging.info(f'Solved!\n updating paths...')
            self._update_paths()
            logging.info(f'Done\n Removing processed tasks...')
            self._remove_processed_tasks()
        self._robot_returns()
        #TODO

    def _solve_viet(self, 
                    total_robots, 
                    consider_process_time):
        starting_times = {}
        for key,value in self.paths.items():
            starting_times[key] = value.total_time
        R_A = self._cal_RA() #robots required to do all the tasks
        if R_A <= total_robots:
            print(f'R_A = {R_A} <= total_robots = {total_robots}, solving first case' )
            self._lastest_solve_result = self._solve_first_case(total_robots, 
                                                                starting_times, 
                                                                consider_process_time)
            self.lastest_case = 1
        else: 
            print(f'R_A = {R_A} > total_robots = {total_robots}, solving second case')
            self._lastest_solve_result = self._solve_second_case_thao(total_robots, 
                                                                 starting_times, 
                                                                 consider_process_time)
            self.lastest_case = 2
            
    def _solve_second_case(self,
                           total_robots,
                           starting_times :dict,
                           consider_process_time : bool):
        
        # total_robots_required = np.sum([task.n for task in self.tasks_remains]) # This is probably for assign all 
                                                                                # robots at each iteration case
        R_A = self._cal_RA() 
        A_idcs = [task.index for task in self.tasks_remains]
        B_idcs = [j for j in range(total_robots)]
        Bf_idcs = [j for j in range(total_robots, R_A)] #Artificial robots
        Bp_idcs = B_idcs + Bf_idcs
        BpA_idcs = [(i,j) for i in Bp_idcs for j in A_idcs]

        a = self._construct_a_artificial(total_robots, R_A)
        t = starting_times
        c = self.c_operate
        n ={task.index: task.n for task in self.tasks_remains}

        m = Model('Viet_case_2')
        C_max = m.continuous_var(name = 'C_max')
        x = m.binary_var_dict(BpA_idcs, name = 'x')
        # x = m.continuous_var_dict(BpA_idcs, ub =1, name = 'x')
        v= m.binary_var_dict(A_idcs, name = 'v')


        m.minimize(C_max)
        if consider_process_time:
            m.add_constraints(C_max >= (a[i][j] + t[i] + c[j])*x[i,j]  for i in B_idcs for j in A_idcs)
        else:
            # m.add_constraints(C_max + (a[i][j] + t[i])*v[j] >= (a[i][j]+ t[i])*x[i,j] for i in B_idcs for j in A_idcs)
            m.add_constraints(C_max  >= (a[i][j]+ t[i])*x[i,j]  for i in B_idcs for j in A_idcs)

        m.add_constraints(m.sum(x[i,j] for i in Bp_idcs) == n[j] for j in A_idcs)
        m.add_constraints(m.sum(x[i,j] for j in A_idcs) <= 1 for i in Bp_idcs)
        m.add_constraints(x[i,j] <= 1 for i in Bp_idcs for j in A_idcs) #Capacity constraint
        m.add_constraint(m.sum((m.sum(x[i,j] for j in A_idcs) for i in Bp_idcs)) == R_A) #Required flow
        m.add_constraints(x[i,j] <= v[j] for i in Bf_idcs for j in A_idcs) # Relation x and v

        #Constraints regarding p
        # m.add_constraints(p[i,j] <= x[i,j] for i in Bp_idcs for j in A_idcs)
        # m.add_constraints(p[i,j] <= v[j] for i in Bp_idcs for j in A_idcs)
        # m.add_constraints(p[i,j] >= x[i,j] + v[j] -1 for i in Bp_idcs for j in A_idcs)

        m.parameters.mip.tolerances.mipgap = cfg.SOLVE_GAP
        m.time_limit = cfg.SOLVE_TIME
        if cfg.EXPORT_MODEL == True:
            print(m.export_as_lp())
        sol = m.solve(log_output = cfg.LOG_OUTPUT)
        # print(sol.display())
        solved_gap = m.solve_details.mip_relative_gap
        processed_nodes = m.solve_details.nb_nodes_processed
        solve_time = m.solve_details.time
        solve_info = {
            'sol':sol,
            'x':x,
            'v':v,
            'processed_nodes': processed_nodes,
            'gap':solved_gap,
            'solve_time':solve_time
        }
        
        return solve_info

    def _solve_second_case_thao(self,
                           total_robots,
                           starting_times :dict,
                           consider_process_time : bool):
        
        # total_robots_required = np.sum([task.n for task in self.tasks_remains]) # This is probably for assign all 
                                                                                # robots at each iteration case
        R_A = self._cal_RA() 
        A_idcs = [task.index for task in self.tasks_remains]
        B_idcs = [j for j in range(total_robots)]
        Bf_idcs = [j for j in range(total_robots, R_A)] #Artificial robots
        Bp_idcs = B_idcs + Bf_idcs
        BpA_idcs = [(i,j) for i in Bp_idcs for j in A_idcs]

        a = self._construct_a_artificial(total_robots, R_A)
        t = starting_times
        c = self.c_operate
        n ={task.index: task.n for task in self.tasks_remains}

        m = Model('Viet_case_2')
        C_max = m.continuous_var(name = 'C_max')
        x = m.binary_var_dict(BpA_idcs, name = 'x')
        # x = m.continuous_var_dict(BpA_idcs, ub =1, name = 'x')
        p =m.continuous_var_dict(BpA_idcs, name = 'p')
        v = m.binary_var_dict(A_idcs, name = 'v')

        m.minimize(C_max)
        if consider_process_time:
            m.add_constraints(C_max >= (a[i][j] + t[i] + c[j])*x[i,j] +m.sum(cfg.BIG_M1*p[i,j] for j in A_idcs) for i in B_idcs for j in A_idcs)
        else:
            m.add_constraints(C_max  >= (a[i][j]+ t[i])*x[i,j] +m.sum(cfg.BIG_M1*p[i,j] for j in A_idcs) for i in B_idcs for j in A_idcs)

        m.add_constraints(m.sum(x[i,j] for i in Bp_idcs) == n[j] for j in A_idcs) 
        m.add_constraints(m.sum(x[i,j] for j in A_idcs) <= 1 for i in Bp_idcs)
        m.add_constraints(x[i,j] <= 1 for i in Bp_idcs for j in A_idcs) 
        m.add_constraint(m.sum((m.sum(x[i,j] for j in A_idcs) for i in Bp_idcs)) == R_A)
        m.add_constraints(x[i,j] <= v[j] for i in Bf_idcs for j in A_idcs) # Relation x and v

        #Constraints regarding p
        m.add_constraints(p[i,j] <= x[i,j] for i in Bp_idcs for j in A_idcs)
        m.add_constraints(p[i,j] <= v[j] for i in Bp_idcs for j in A_idcs)
        m.add_constraints(p[i,j] >= x[i,j] + v[j] -1 for i in Bp_idcs for j in A_idcs)

        m.parameters.mip.tolerances.mipgap = cfg.SOLVE_GAP
        m.time_limit = cfg.SOLVE_TIME
        if cfg.EXPORT_MODEL == True:
            print(m.export_as_lp())
        sol = m.solve(log_output = cfg.LOG_OUTPUT)
        # print(sol.display())
        solved_gap = m.solve_details.mip_relative_gap
        processed_nodes = m.solve_details.nb_nodes_processed
        solve_time = m.solve_details.time
        solve_info = {
            'sol':sol,
            'x':x,
            'v':v,
            'processed_nodes': processed_nodes,
            'gap':solved_gap,
            'solve_time':solve_time
        }
        
        return solve_info
    
    def _solve_first_case(self, 
                          total_robots, 
                          starting_times :dict, 
                          consider_process_time : bool):
        A_idcs = [task.index for task in self.tasks_remains]
        B_idcs = [j for j in range(total_robots)]
        BA_idcs = [(i,j) for i in B_idcs for j in A_idcs]

        a = self._construct_a(total_robots)
        t = starting_times
        c = self.c_operate
        n ={task.index: task.n for task in self.tasks_remains}

        m = Model('Viet_case_1')

        C_max = m.continuous_var(name = 'C_max')
        x = m.binary_var_dict(BA_idcs,ub=1, name = 'x')

        m.minimize(C_max)

        if consider_process_time:
            m.add_constraints(t[i] + a[i][j]*x[i,j] +c[j] <= C_max for i in B_idcs for j in A_idcs)
        else:
            m.add_constraints(t[i] + a[i][j]*x[i,j] <= C_max for i in B_idcs for j in A_idcs)

        m.add_constraints(m.sum(x[i,j] for i in B_idcs) == n[j] for j in A_idcs)
        m.add_constraints(m.sum(x[i,j] for j in A_idcs) <= 1 for i in B_idcs)

        m.parameters.mip.tolerances.mipgap = cfg.SOLVE_GAP
        m.time_limit = cfg.SOLVE_TIME
        if cfg.EXPORT_MODEL == True:
            print(m.export_as_lp())
        sol = m.solve(log_output = cfg.LOG_OUTPUT)

        solved_gap = m.solve_details.mip_relative_gap
        processed_nodes = m.solve_details.nb_nodes_processed
        solve_time = m.solve_details.time
        solve_info = {
            'sol':sol,
            'x':x,
            'processed_nodes': processed_nodes,
            'gap':solved_gap,
            'solve_time':solve_time
        }
        
        return solve_info
    

class VGeneralizedSolver(VietSolver):
    
    def solve(self, total_robots, tasks_each_iteration, consider_process_time, **kwargs):
        "Do not compare process time"
        depot = self.processed.depot
        tasks = self.processed.tasks

        c_travel = copy.deepcopy(self.c_travel)
        c_operate = copy.deepcopy(self.c_operate)
        c_travel_back  ={}
        for i in c_travel:
            try:
                c_travel[i].pop(i)
                c_travel_back[i] = c_travel[i].pop(0)
            except:
                pass


        uninserted = copy.deepcopy(tasks) #j left
        uninserted_idx = [task.index for task in tasks] #J_t
        paths = [Path(depot) for _ in range(total_robots)]
        cur_sol = Solution(paths, depot, tasks)

        idx_task_mapper = {task.index: task for task in uninserted}
        C_max = 0
        inserted_idcs = []
        while len(uninserted) > 0:
            inserted_paths = []
            L ={}
            for task in uninserted:
                task_idx = task.index
                L[task_idx] = {} 
                for path in cur_sol.paths:
                    if not consider_process_time:
                        cost_if_insert = path.get_start_insert_cost(task, c_travel)
                    else:
                        cost_if_insert = path.get_insert_cost(task, c_travel)
                    L[task_idx][path] = cost_if_insert
            L_list = [(task_idx,path,cost) for task_idx in L for path,cost in L[task_idx].items()]
            df_L = pd.DataFrame(L_list, columns = ['task_idx', 'path', 'cost']).sort_values('cost')
            sorted_L_list = df_L.values.tolist()
            
            #Declare variables
            remaining_robots = total_robots
            cur_n = {task.index:0 for task in uninserted}
            best_paths = {task.index:[] for task in uninserted}
            cur_task = {path:[] for path in paths}

            if type(tasks_each_iteration) == int:
                k = tasks_each_iteration
            elif tasks_each_iteration == 'random':
                task_range = kwargs.get('task_range')
                k = np.random.randint(task_range[0],task_range[1])
                
            #Choose the task
            for item in sorted_L_list:
                task_idx, path, cost_if_insert = item #cost if insert
                task = idx_task_mapper[task_idx]
                if task.n > remaining_robots or task_idx in inserted_idcs or path in inserted_paths:
                    continue
                cur_n[task_idx] += 1
                best_paths[task_idx].append(path)
                cur_task[path].append(task_idx)
                
                if cur_n[task_idx] == task.n:
                    if not consider_process_time:
                        c_max = cost_if_insert + c_operate[task.index]
                    if consider_process_time:
                        c_max = cost_if_insert
                    for path in best_paths[task_idx]:
                        path.insert(task)
                        path.update_total_time(c_max)
                        inserted_paths.append(path)
                        for task_prime_idx in cur_task[path]:
                            if task_prime_idx != task_idx:
                                best_paths[task_prime_idx].remove(path)
                                cur_n[task_prime_idx] = cur_n[task_prime_idx] - 1
                    uninserted.remove(task)
                    # print(f'\n path length {[(path,path.total_time) for path in paths]}')
                    inserted_idcs.append(task_idx)
                    remaining_robots -= task.n
                    k = k - 1
                    if k == 0:
                        break
            

        # insert 0 to path
        for path in paths:
            last_idx = path.get_last_task().index
            path.add_total_time(c_travel_back[last_idx])
            path.insert(depot)

        print(f'Viet generalized sol is {cur_sol.get_makespan().round(2)}')

        return cur_sol

class JPGeneralizedSolver(VGeneralizedSolver):
    """JP want to insert the task to the path with the minimum cost with rate 80%, 
    second with 15%, and third with 5%
    """

    def solve(self, total_robots, tasks_each_iteration, consider_process_time, **kwargs):
        "Do not compare process time"
        depot = self.processed.depot
        tasks = self.processed.tasks

        c_travel = copy.deepcopy(self.c_travel)
        c_operate = copy.deepcopy(self.c_operate)
        c_travel_back  ={}
        for i in c_travel:
            try:
                c_travel[i].pop(i)
                c_travel_back[i] = c_travel[i].pop(0)
            except:
                pass


        uninserted = copy.deepcopy(tasks) #j left
        uninserted_idx = [task.index for task in tasks] #J_t
        paths = [Path(depot) for _ in range(total_robots)]
        cur_sol = Solution(paths, depot, tasks)

        idx_task_mapper = {task.index: task for task in uninserted}
        C_max = 0
        inserted_idcs = []
        while len(uninserted) > 0:
            L ={}
            for task in uninserted:
                task_idx = task.index
                L[task_idx] = {} 
                for path in cur_sol.paths:
                    if not consider_process_time:
                        cost_if_insert = path.get_start_insert_cost(task, c_travel)
                    else:
                        cost_if_insert = path.get_insert_cost(task, c_travel)
                    L[task_idx][path] = cost_if_insert
            L_list = [(task_idx,path,cost) for task_idx in L for path,cost in L[task_idx].items()]
            df_L = pd.DataFrame(L_list, columns = ['task_idx', 'path', 'cost']).sort_values('cost')
            sorted_L_list = df_L.values.tolist()
            
            #Declare variables
            remaining_robots = total_robots
            cur_n = {task.index:0 for task in uninserted}
            best_paths = {task.index:[] for task in uninserted}
            cur_task = {path:[] for path in paths}
                
            #Choose the task
            choose_ratio = [0.8, 0.15, 0.05]
            choose_rank = np.random.choice([0,1,2], p = choose_ratio)
            # print(f'choose rank {choose_rank}')
            cur_rank = 0
            for item in sorted_L_list:
                task_idx, path, cost_if_insert = item #cost if insert
                task = idx_task_mapper[task_idx]

                cur_n[task_idx] += 1
                best_paths[task_idx].append(path)
                cur_task[path].append(task_idx)
                if cur_n[task_idx] == task.n:
                    # logging.info(f'{cur_rank} rank: {task_idx}')
                    if cur_rank < choose_rank:
                        cur_rank += 1
                        continue
                    if not consider_process_time:
                        c_max = cost_if_insert + c_operate[task.index]
                    if consider_process_time:
                        c_max = cost_if_insert
                    for path in best_paths[task_idx]:
                        path.insert(task)
                        path.update_total_time(c_max)
                    uninserted.remove(task)
                    break
            # print("Inserted",{task_idx})
            

        # insert 0 to path
        for path in paths:
            last_idx = path.get_last_task().index
            path.add_total_time(c_travel_back[last_idx])
            path.insert(depot)

        print(f'Viet generalized sol is {cur_sol.get_makespan().round(2)}')

        return cur_sol

class TabuSearch:

    def __init__(self, processed, c_travel, c_operate, init_solution_func = VGeneralizedSolver, **kwargs):
        self.processed = processed
        self.c_travel = c_travel
        self.c_operate = c_operate
        self.init_solution_func = init_solution_func
        self.init_solution_path = kwargs.get('init_solution_path', None)

    def _get_fitness(self, graph):
        makespan = self._get_start_time(graph, self.c_travel, self.c_operate)[-1]
        # fitness = 1/(makespan+0.000001)
        fitness = makespan
        return fitness

    def _get_neighbors(self,graph, affected_add, affected_drop,affected_redistribute):
        neighbors = []
        time_0 = time.time()
        neighbors += self._get_n_reroute_neighbors(
            graph, cfg.TABU_NUM_REROUTE_NEIGHBORS, affected_add, affected_drop)
        time_1 = time.time()
        neighbors += self._get_n_reverse_neighbors(
            graph, cfg.TABU_NUM_REVERSE_NEIGHBORS, affected_add, affected_drop)
        time_2 = time.time()
        neighbors += self._get_n_redistribute_neighbors(
            graph,cfg.TABU_NUM_REDISTRIBUTE_NEIGHBORS, affected_redistribute)
        time_3 = time.time()
        logging.info(f'get neighbors time:\n reroute {(time_1-time_0)/cfg.TABU_NUM_REROUTE_NEIGHBORS} \n reverse {(time_2-time_1)/cfg.TABU_NUM_REVERSE_NEIGHBORS} \n redistribute {(time_3-time_2)/cfg.TABU_NUM_REVERSE_NEIGHBORS}')

        return neighbors


    def _redistribute(self, graph: nx.DiGraph, topo_order :list, cut_point :int, Ls):
        """Alain's idea of cut and redistribute

        Parameters
        ----------
        graph : nx.DiGraph
            graph representation of the solution
        topo_order : _type_
            _description_
        cut_point : _type_
            _description_
        """
        #cut
        #calculate pi_i left and right
        N_left = topo_order[:cut_point]
        N_right = topo_order[cut_point:]
        pi_left = {node: np.sum(graph.edges[node,succ]['weight'] for succ in graph.successors(node) if succ in N_right) for node in N_left}
        pi_right = {node: np.sum(graph.edges[pred,node]['weight'] for pred in graph.predecessors(node) if pred in N_left) for node in N_right}
        result_graph = copy.deepcopy(graph)
        edges = list(graph.edges())
        for edge in edges:
            if edge[0] in N_left and edge[1] in N_right:
                result_graph.remove_edge(edge[0],edge[1])

        solve_results = self._dedicated_redistribute(result_graph, N_left, N_right, pi_left, pi_right, Ls)
        f = solve_results['sol'].get_value_dict(solve_results['f'], keep_zeros = False)
        for key, value in f.items():
            result_graph.add_edge(key[0],key[1], weight = value)
        return result_graph
    
    def _dedicate_redistribute(self, graph: nx.DiGraph, topo_order :list, cut_point :int, Ls):
        """Alain's idea of cut and redistribute

        Parameters
        ----------
        graph : nx.DiGraph
            graph representation of the solution
        topo_order : _type_
            _description_
        cut_point : _type_
            _description_
        """
        #cut
        #calculate pi_i left and right
        N_left = topo_order[:cut_point]
        N_right = topo_order[cut_point:]
        pi_left = {node: np.sum(graph.edges[node,succ]['weight'] for succ in graph.successors(node) if succ in N_right) for node in N_left}
        pi_right = {node: np.sum(graph.edges[pred,node]['weight'] for pred in graph.predecessors(node) if pred in N_left) for node in N_right}
        result_graph = deep_copy_graph(graph)

        result_graph = self._dedicated_redistribute(result_graph, N_left, N_right, pi_left, pi_right, Ls)
        return result_graph

    def _cplex_redistribute(self, N_left, N_right, pi_left, pi_right, Ls):
        L, L_bar, L_star = Ls
        c_travel = self.c_travel

        m = Model('redistribute')
        N_index = [(i,j) for i in N_left for j in N_right]
        x = m.binary_var_dict(N_index, name = 'x')
        f = m.continuous_var_dict(N_index, name = 'f')
        C_max = m.continuous_var(name = 'C_max')

        m.minimize(C_max)
        m.add_constraints((L_bar[i] + c_travel[i][j] + L_star[j])*x[i,j] <= C_max for i in N_left for j in N_right)
        m.add_constraints(m.sum(f[i,j] for j in N_right) == pi_left[i] for i in N_left)
        m.add_constraints(m.sum(f[i,j] for i in N_left) == pi_right[j] for j in N_right)
        m.add_constraints(f[i,j] <= 30*x[i,j] for i in N_left for j in N_right)
        m.add_constraints(f[i,j] >= x[i,j] for i in N_left for j in N_right)
        
        cplex_sol = m.solve(log_output = cfg.LOG_OUTPUT)
        # print(cplex_sol.objective_value) #TODO delete
        if cfg.EXPORT_MODEL == True:
            m.export_as_lp()
        solved_gap = m.solve_details.mip_relative_gap
        processed_nodes = m.solve_details.nb_nodes_processed
        solve_time = m.solve_details.time

        solve_info = {
            'sol':cplex_sol,
            'x':x,
            'f':f,
            'processed_nodes': processed_nodes,
            'gap':solved_gap,
            'solve_time':solve_time
        }
        return solve_info

    def _dedicated_redistribute(self, sol_graph, N_left, N_right, pi_left, pi_right, Ls):
        """
        Generate a new solution by redistributing the tasks in N_left to N_right
        using Alain's idea
        """
        start_time = time.time()
        l, l_bar, l_star = Ls
        C_max = l[-1]
        sol_graph = deep_copy_graph(sol_graph)
        deltas = {
            (i,j): l_bar[i] + self.c_travel[i][j] + l_star[j] for i in N_left for j in N_right
            }
        improved = False
        stop = False
        while not stop:
            # f = {(i,j): sol_graph[i][j]['weight'] for i in N_left for j in N_right if pi_left[i] > 0 and pi_right[j] > 0}
            pi_left = {node: np.sum(sol_graph.edges[node,succ]['weight'] for succ in sol_graph.successors(node) if succ in N_right) for node in N_left}
            pi_right = {node: np.sum(sol_graph.edges[pred,node]['weight'] for pred in sol_graph.predecessors(node) if pred in N_left) for node in N_right}
            f = self._build_f(sol_graph, N_left, N_right, pi_left, pi_right)
            C_max = min(C_max,max(deltas[(i,j)] for i in N_left for j in N_right if (i,j) in f))
            Af= [(i,j) for i in N_left for j in N_right  if pi_left[i] > 0 and pi_right[j] > 0 and deltas[(i,j)] >= C_max]
            A_max = [edge for edge in Af if edge in f]

            if len(A_max) > 0:
                selected_edge = random.choice(A_max)
            (i_star,j_star) = selected_edge

            gamma = self._find_gamma(N_left, N_right, Af, f, i_star, j_star)
            if gamma == None:
                stop = True
            else:
                # Update f
                delta_f = self._redis_cal_delta_flows(gamma, f)
                sol_graph = self._redis_update_graph(sol_graph, gamma, delta_f)
                # Update C_max
                new_C_max = max(deltas[(i,j)] for i,j in f)
                if new_C_max < C_max:
                    C_max = new_C_max
                    improved = True
        end_time = time.time()
        logging.info(f'Redistribute time {end_time - start_time}')
        return sol_graph
    
    def _build_f(self, sol_graph, N_left, N_right, pi_left, pi_right):
        """
        Return flow value between every pair in N_left and N_right
        """
        f = {}
        for i in N_left:
            if pi_left[i] > 0:
                for j in N_right:
                    if pi_right[j] > 0:
                        try:
                            f[i,j] = sol_graph[i][j]['weight']
                        except:
                            pass
        return f

    def _find_gamma(self, N_left, N_right, Af, f, i_star, j_star):
        
        queue = deque([([i_star], i_star, None)])  # Start with i_star, no j yet
        visited = set()  # To avoid revisiting the same i, j combinations

        while queue:
            path, current_i, current_j = queue.popleft()

            # If the last node in the path is j_star, return the found path
            if current_j == j_star:
                return path

            if current_j is None:
                # Initial step: find j_1 such that (i_star, j_1) is not in Af
                for j in N_right:
                    if (i_star, j) not in Af:
                        new_path = path + [j]  # Add j to path
                        queue.append((new_path, i_star, j))
                        visited.add((i_star, j))
            else:
                # Intermediate steps: find valid (i_k, j_k) pairs
                for i_next in N_left:
                    if (i_next, current_j) in f and (i_next, current_j) not in Af:
                        for j_next in N_right:
                            if (i_next, j_next) not in Af and (i_next, j_next) not in visited:
                                new_path = path + [i_next, j_next]  # Add both i and j to path
                                queue.append((new_path, i_next, j_next))
                                visited.add((i_next, j_next))

        # If no valid path is found
        
        return None
    
    def _redis_cal_delta_flows(self, gamma, f) -> int:
        """Generate maximum number of flows to be redistributed

        Parameters
        ----------
        gamma : list[int]
            The path from i_star to j_star
        f : dict
            _description_

        Returns
        -------
        int
            number of flows to be redistributed
        """
        affected_edges = [(gamma[2*k], gamma[2*k - 1]) for k in range(1, len(gamma)//2)]
        affected_flows = []
        for edge in affected_edges:
            affected_flows.append(f[edge])
        longest_edge = f[(gamma[0], gamma[-1])]
        affected_flows.append(longest_edge)

        return min(affected_flows)
        
    def _redis_update_graph(self, graph, gamma, delta_f):
        result_graph = deep_copy_graph(graph)
        decrease_edges = [(gamma[2*k], gamma[2*k - 1]) for k in range(1, len(gamma)//2)]
        decrease_edges.append((gamma[0], gamma[-1]))
        increase_edges = [(gamma[2*k], gamma[2*k + 1]) for k in range(1, len(gamma)//2 - 1)] #TODO check off by 1
        increase_edges += [(gamma[0], gamma[1]), (gamma[-2], gamma[-1])] # (i_star,j_1) and (i_{K},j_star)

        for edge in decrease_edges:
            result_graph[edge[0]][edge[1]]['weight'] -= delta_f
            if result_graph[edge[0]][edge[1]]['weight'] == 0:
                result_graph.remove_edge(edge[0], edge[1])
        
        for edge in increase_edges:
            if result_graph.has_edge(edge[0], edge[1]):
                result_graph[edge[0]][edge[1]]['weight'] += delta_f
            else:
                result_graph.add_edge(edge[0], edge[1], weight = delta_f)

        return result_graph

    def _get_n_redistribute_neighbors(self,graph, num_neighbors, affected_redistribute):
        start_time = time.time()
        topo_order = list(nx.topological_sort(graph))
        z, w = self._get_related_times(graph, self.c_travel, self.c_operate) # z is L
        # calculate L, L_bar, L_star
        L = z
        L_bar = {node: L[node] + self.c_operate[node] for node in L}
        L_star = self._get_longest_path(graph, self.c_travel, self.c_operate)
        Ls = (L, L_bar, L_star)
        total_nodes = len(topo_order)
        # cuts_after = np.random.default_rng(cfg.RND_SEED).choice(range(2,total_nodes-2), size = num_neighbors, replace=False)
        cuts_after = np.random.choice(range(2,total_nodes-2), size = num_neighbors, replace=False)

        neighbors = []
        for i in cuts_after:
            neighbor = self._dedicate_redistribute(graph,topo_order, i, Ls)
            neighbors.append(neighbor)
            affected_redistribute[neighbor] = i
        return neighbors

    def _get_n_reverse_neighbors(self, graph, max_neighbors, affected_add, affected_drop, on_critical = cfg.TABU_REVERSE_ON_CRITICAL):
        start_time = time.time()
        neighbors = []
        # ni_descendants is a dictionary of descendants that are not neighbor of each node in nx Digraph
        ni_descendants = {}
        for node in graph.nodes:
            successors = list(graph.successors(node))
            descendants = []
            for succ in successors:
                neighbor_descendants = list(nx.descendants(graph,succ))
                descendants += neighbor_descendants
            ni_descendants[node] = set(descendants)
        related_times = self._get_related_times(graph, self.c_travel, self.c_operate)

        if on_critical:
            edges = self._get_critical_edges(graph,self.c_travel,self.c_operate)
        else:
            edges = list(graph.edges())

        np.random.shuffle(edges)
        num_neighbors = 0
        for edge in edges:
            if num_neighbors == max_neighbors:
                return neighbors
            if self._reversable(edge, ni_descendants):
                G_p = self._reverse(edge, graph, related_times)
                neighbors.append(G_p)
                affected_add[G_p] = [edge]
                affected_drop[G_p] = [(edge[1],edge[0])]
                num_neighbors += 1
        
        end_time = time.time()
        return neighbors

    def _get_u(self, edge, graph, related_times):
        """Get u_i in reverse neighborhood

        Parameters
        ----------
        pred : _type_
            _description_
        weight_pred : _type_
            _description_
        waiting_pred : _type_
            _description_
        reverse_weight : _type_
            _description_

        Returns
        -------
        _type_
            _description_
        """
        start_time, waiting_time = related_times
        (a,b) = edge
        
        pred_a = list(graph.predecessors(a))
        weight_pred_a = {pred:graph[pred][a]['weight'] for pred in pred_a}
        waiting_pred_a = {pred:waiting_time[(pred,a)] for pred in pred_a}
        df = pd.DataFrame(columns=['pred','waiting_time', 'weight'])
        weights = []
        waitings = []
        for i in pred_a:
            weights.append(weight_pred_a[i])
            waitings.append(waiting_pred_a[i])
        df['pred'] = pred_a
        df['waiting_time'] = waitings
        df['weight'] = weights
        df = df.sort_values(by='waiting_time', ascending = False)

        sum_weight = 0
        u_a = []
        i = 0
        reverse_weight = graph[a][b]['weight']
        for weight in df['weight']:
            sum_weight += weight
            i += 1
            if sum_weight >= reverse_weight:
                break
        u_a = list(df['pred'][:i])
            
        return u_a
    
    def _get_v(self, edge, graph, related_times):
        """Get u_i in reverse neighborhood

        Parameters
        ----------
        pred : _type_
            _description_
        weight_pred : _type_
            _description_
        waiting_pred : _type_
            _description_
        reverse_weight : _type_
            _description_

        Returns
        -------
        _type_
            _description_
        """
        start_time, waiting_time = related_times
        (a,b) = edge

        succ_b = list(graph.successors(b))
        weight_succ_b = {succ:graph[b][succ]['weight'] for succ in succ_b}

        df = pd.DataFrame(columns=['pred','waiting_time', 'weight'])
        weights = []
        succ_c_travel = []
        for i in succ_b:
            weights.append(weight_succ_b[i])
            succ_c_travel.append(self.c_travel[b][i])
        df['succ'] = succ_b    
        df['c_travel'] = succ_c_travel
        df['weight'] = weights
        
        df = df.sort_values(by='c_travel', ascending = True)
        sum_weight = 0
        chosen_preds = []
        i = 0
        reverse_weight = graph[a][b]['weight']
        for weight in df['weight']:
            sum_weight += weight
            i += 1
            if sum_weight >= reverse_weight:
                break
        chosen_preds = list(df['succ'][:i])
        return chosen_preds

    def _reverse(self,edge, graph, related_times):
        start_time = time.time()
        u_a = self._get_u(edge, graph, related_times)
        v_b = self._get_v(edge, graph, related_times)
        graph_prime = deep_copy_graph(graph)
        (a,b) = edge
        reverse_weight = graph_prime[a][b]['weight']

        weight_to_remove = reverse_weight
        for u in u_a:
            weight_ua = graph_prime[u][a]['weight']
            cur_weight_to_remove = min(weight_ua, weight_to_remove)
            graph_prime[u][a]['weight'] -= cur_weight_to_remove
            if graph_prime[u][a]['weight'] == 0:
                graph_prime.remove_edge(u,a)
            if (u,b) in graph_prime.edges():
                graph_prime[u][b]['weight'] += cur_weight_to_remove
            else:
                graph_prime.add_edge(u,b, weight = cur_weight_to_remove)
            weight_to_remove -= weight_ua
        
        weight_to_remove = reverse_weight
        for v in v_b:
            weight_bv = graph_prime[b][v]['weight']
            cur_weight_to_remove = min(weight_bv, weight_to_remove)
            graph_prime[b][v]['weight'] -= cur_weight_to_remove
            if graph_prime[b][v]['weight'] == 0:
                graph_prime.remove_edge(b,v)
            if (a,v) in graph_prime.edges():
                graph_prime[a][v]['weight'] += cur_weight_to_remove
            else:
                graph_prime.add_edge(a,v, weight = cur_weight_to_remove)
            weight_to_remove -= weight_bv
        graph_prime.remove_edge(a,b)
        graph_prime.add_edge(b,a, weight = reverse_weight)
        end_time = time.time()
        # logging.info(f'reverse time {end_time - start_time}')
        return graph_prime
    
    def _reversable(self, edge, ni_descendants):
        """Check if an edge can be reversed,

        Parameters
        ----------
        graph : nx.DiGraph or nx.MultiDiGraph
            the initial graph
        edge : tuple (int,int)
            the edge to be check if it can be reversed
        ni_descendants : dict
            dictionary of non immediate descendants of each node
        """
        (a,b) = edge
        if b in ni_descendants[a]:
            return False
        if b == -1:
            return False
        if a == 0:
            return False
        else:
            return True

    def _rerouteable(self, edge1, edge2, descendants):
        """
        Check if 2 edges can be applied 2-opt
        """
        (a,b) = edge1
        (c,d) = edge2
            
        if a==d or a==c or b==c or b==d:
            return False
        elif a in descendants[d]:
            return False
        elif c in descendants[b]:
            return False
        else:
            return True
     
    def _reroute(self,graph, edge1, edge2):
        (a,b) = edge1
        (c,d) = edge2
        w1 = graph[a][b]['weight']
        w2 = graph[c][d]['weight']
        reroute_amount = min(w1,w2)

        graph[a][b]['weight'] -= reroute_amount
        if graph[a][b]['weight'] == 0:
            graph.remove_edge(a,b)

        graph[c][d]['weight'] -= reroute_amount
        if graph[c][d]['weight'] == 0:
            graph.remove_edge(c,d)

        if graph.has_edge(a,d):
            graph[a][d]['weight'] += reroute_amount
        else:
            graph.add_edge(a,d,weight = reroute_amount)

        if graph.has_edge(c,b):
            graph[c][b]['weight'] += reroute_amount
        else:
            graph.add_edge(c,b,weight = reroute_amount)
        end_time = time.time()
        return graph

    def _get_n_reroute_neighbors(
            self,
            graph, 
            max_neighbors, 
            affected_add, 
            affected_drop, 
            on_critical = cfg.TABU_REROUTE_ON_CRITICAL,
            longest_dist =cfg.TABU_REROUTE_LONGEST_EDGE):
        # This is N_reroute in the paper
        neighbors = []
        descendants = {node: nx.descendants(graph,node) for node in graph.nodes}
        if on_critical:
            edges_i = self._get_critical_edges(graph,self.c_travel,self.c_operate)
            if longest_dist:
                edges_i = self._get_longest_critical_edge(edges_i,self.c_travel)
        else:
            edges_i = list(graph.edges())
        edges_j = list(graph.edges())
        np.random.shuffle(edges_i)
        np.random.shuffle(edges_j)
        num_neighbors = 0
        for i in range(len(edges_i)):
            for j in range(len(edges_j)):
                if num_neighbors == max_neighbors:
                    return neighbors
                edge1 = edges_i[i]
                edge2 = edges_j[j]
                if self._rerouteable(edge1, edge2, descendants):
                    G_p = copy.deepcopy(graph)
                    G_p = self._reroute(G_p, edge1, edge2)
                    neighbors.append(G_p)
                    affected_add[G_p] = [edge1, edge2]
                    affected_drop[G_p] = [(edge1[0],edge2[1]),(edge2[0],edge1[1])]
                    num_neighbors += 1
        return neighbors

    def _path_to_edges(self, path : list):
        edges = []
        cur_len = len(path)
        if path[0] != 0:
            edges.append((0,path[0]))
        for i in range(cur_len - 1):
            edges.append((path[i],path[i + 1]))
        # edges.append((path[-1],-1))
        return edges

    def _paths_to_weighted_edges(self,paths: list): 
        """
        extract the edges to the directed graph G from a
        set of paths. More on section 2 in "Formal construct
        solution from paths 13-12-2023" document
        
        Parameters
        ----------
        paths : list
            list of tuples, every tuple is a path of every robot
            excluding the sink and the source (nessessary?)
            
        Returns
        -------
        edges : list
            list of weighted edges, every element has the form of (source, sink, weight), the weight is number of flow traverse the edge
            
        """
        edges = []
        for path in paths:
            cur_edges = self._path_to_edges(path)
            edges += cur_edges
        # edges = dict((x,edges.count(x)) for x in set(edges))
        edges = [(edge[0],edge[1], edges.count(edge)) for edge in set(edges)]
        return edges

    def _get_out_nodes(self,graph):
        """
        Get a dictionary of out node from all nodes on a directed graph
        
        Parameters
        ----------
        graph : nx.DiGraph or nx.MultiDiGraph


        Returns
        -------
        out_nodes : dict
            `out_nodes[1] = [2,3,69]` means that edges (1,2), (1,3), (1,69) in the graph.
        """
        out_nodes = {}
        nodes = list(graph.nodes())
        for node in nodes:
            out_nodes[node] = list(graph.successors(node))

        return out_nodes
    
    def _get_in_nodes(self,graph):
        """
        Get a dictionary of in nodes from all nodes on a directed graph
        
        Parameters
        ----------
        graph : nx.DiGraph or nx.MultiDiGraph


        Returns
        -------
        in_nodes : dict
            `in_nodes[35] = [2,3,69]` means that edges (2,35), (3,35), (69,35) are in the graph.
        """
        in_nodes = {}
        nodes = list(graph.nodes())
        for node in nodes:
            in_nodes[node] = list(graph.predecessors(node))

        return in_nodes
    
    def _get_start_time(self, graph, c_travel, c_operate):
        """Get the start time of each task in the graph

        Parameters
        ----------
        graph : _type_
            _description_
        c_travel : _type_
            _description_
        c_operate : _type_
            _description_

        Returns
        -------
        _type_
            _description_
        """
        reversed_topo_order = list(reversed(list(nx.topological_sort(graph))))
        out_nodes = self._get_out_nodes(graph)

        
        c_operate[-1] = 0
        c_travel[-1] = {}
        c_travel[0][-1] = 0
        for i in c_travel:
            if i!= 0 and i!=-1:
                c_travel[i][-1] = c_travel[i][0]
        
        z = {}
        for j in c_operate:
            z[j] = -np.infty
        z[0] = 0
        z[-1] = -np.infty
        
        while reversed_topo_order:
            cur_task = reversed_topo_order.pop()
            for succ in out_nodes[cur_task]:
                z_cur = z[cur_task] + c_operate[cur_task] + c_travel[cur_task][succ]
                if z[succ] < z_cur:
                    z[succ] = z_cur
        return z
    
    def _get_related_times(self, graph, c_travel, c_operate):
        """Get start time, waiting time of each task in the graph

        Parameters
        ----------
        graph : _type_
            _description_
        c_travel : _type_
            _description_
        c_operate : _type_
            _description_

        Returns
        -------
        _type_
            _description_
        """
        reversed_topo_order = list(reversed(list(nx.topological_sort(graph))))
        out_nodes = self._get_out_nodes(graph)
        c_operate[-1] = 0
        c_travel[-1] = {}
        c_travel[0][-1] = 0
        for i in c_travel:
            if i!= 0 and i!=-1:
                c_travel[i][-1] = c_travel[i][0]
        
        z = {} #z[i] is the start time of task i
        for j in c_operate:
            z[j] = -np.infty
        z[0] = 0
        z[-1] = -np.infty
        while reversed_topo_order:
            cur_task = reversed_topo_order.pop()
            for succ in out_nodes[cur_task]:
                z_cur = z[cur_task] + c_operate[cur_task] + c_travel[cur_task][succ]
                if z[succ] < z_cur:
                    z[succ] = z_cur
        
        edges = list(graph.edges())
        waits = {} # w[(a,b)] is the waitng time of robots start from taks a to task b on task b
        for edge in edges:
            (a,b) = edge
            waits[edge] = z[b] - z[a] - c_operate[a] - c_travel[a][b] 
        return z, waits

    def _get_longest_path(self, graph, c_travel, c_operate):
        """Get longest path from each node to the sink node
        Parameters
        ----------
        graph : _type_
            _description_
        c_travel : _type_
            _description_
        c_operate : _type_
            _description_

        Returns
        -------
        _type_
            _description_
        """
        topo_order = list(nx.topological_sort(graph))
        in_nodes = self._get_in_nodes(graph)
        
        c_operate[-1] = 0
        c_travel[-1] = {}
        c_travel[0][-1] = 0
        for i in c_travel:
            if i!= 0 and i!=-1:
                c_travel[i][-1] = c_travel[i][0]
        
        L_star = {}
        for j in c_operate:
            L_star[j] = -np.infty
        L_star[-1] = 0
        L_star[0] = -np.infty
        
        while topo_order:
            cur_task = topo_order.pop()
            for pred in in_nodes[cur_task]:
                cur_L_star = L_star[cur_task] + c_operate[pred] + c_travel[pred][cur_task]
                if L_star[pred] < cur_L_star:
                    L_star[pred] = cur_L_star

        return L_star
    
    def _get_critical_edges(self, graph_sol: nx.DiGraph, c_travel, c_operate:dict):
        z,w = self._get_related_times(graph_sol, c_travel, c_operate)
        # Find the critical path of graph_sol(nx.DiGraph) and return the path, the start time of each task is in z[i] is the start time of task i,w[(i,j)] is the waiting time of task i to j
        # Get predessors of node -1 in graph_sol (the lastest task in the graph_sol)
        cur_task = -1
        critical_edges = []
        while cur_task !=0:
            cur_preds = list(graph_sol.predecessors(cur_task))
            for pred in cur_preds:
                cur_w  = w[(pred,cur_task)]
                #compare cur_w with 0 with a small epsilon
                if np.abs(cur_w) <= 0.0001:
                    critical_edges.append((pred,cur_task))
                    cur_task = pred
                    break
        return critical_edges

    def _get_longest_critical_edge(self,edges,c_travel):
        longest_edge = None
        longest_dist = -np.infty
        for edge in edges:
            (a,b) = edge
            if c_travel[a][b] > longest_dist:
                longest_dist = c_travel[a][b]
                longest_edge = edge
        return [longest_edge]
    
    def _to_graph(self, paths: list):
        """
        Return a networkx *weighted* graph from the set of paths.
        """
        G = nx.DiGraph()
        weighted_edges = self._paths_to_weighted_edges(paths)
        G.add_weighted_edges_from(weighted_edges)
        return G

    def _init_solution(self,total_robots):
        if self.init_solution_func is VGeneralizedSolver:
            init_instance = self.init_solution_func(
                self.processed, 
                self.c_travel, 
                self.c_operate)
            
            cur_best_makespan = np.infty
            for i in range(cfg.VIET_RANDOM_ITE):
                solve_viet = VGeneralizedSolver(self.processed, self.c_travel, self.c_operate)
                cur_sol = solve_viet.solve(total_robots,'random',cfg.TABU_INIT_CONSIDER_PROCESS_TIME,task_range=[1,4])
                makespan = cur_sol.get_makespan()
        
                if makespan < cur_best_makespan:
                    init_sol  = cur_sol
                    cur_best_makespan = makespan
            
        elif self.init_solution_func is GreedyInsert:
            init_instance = self.init_solution_func(
                self.processed,
                self.c_travel,
                self.c_operate
            )
            init_sol, makespan = init_instance.solve(total_robots)
        
        elif self.init_solution_func is SolutionFetcher:
            init_instance = self.init_solution_func(self.init_solution_path)
            init_sol = init_instance.fetch()

        else:
            init_sol = self.init_solution_func(
                self.processed,
                total_robots
        )

        return init_sol
    

    def _is_tabu(self, affected_add, affected_drop, tl_add, tl_drop):
        if len(affected_add) == 2: #reroute
            (a,b),(c,d) = affected_add
            if (a,d) in tl_add or (c,b) in tl_add:
                return True
            if (a,b) in tl_drop or (c,d) in tl_drop:
                return True
        if len(affected_add) == 1: #reverse
            (a,b) = affected_add[0]
            if (b,a) in tl_add:
                return True
            if (a,b) in tl_drop:
                return True
        
        return False
    
    def _is_tabu_redistribute(self, affected_redistribute, tl_redistribute):
        is_tabu = False
        if affected_redistribute in tl_redistribute:
            is_tabu = True
        return is_tabu

    def _log_config(self):
        if cfg.TABU_LOG_PARAMETERES:
            logging.info(f'TABU SEARCH PARAMETERS')
            logging.info(f'RANDOM_SEED: {cfg.RND_SEED}')
            logging.info(f'NUM_REVERSE_NEIGHBORS: {cfg.TABU_NUM_REVERSE_NEIGHBORS}')
            logging.info(f'NUM_REROUTE_NEIGHBORS: {cfg.TABU_NUM_REROUTE_NEIGHBORS}')
            logging.info(f'NUM_REDISTRIBUTE_NEIGHBORS: {cfg.TABU_NUM_REDISTRIBUTE_NEIGHBORS}')
            logging.info(f'MAX_TL_ADD_SIZE: {cfg.TABU_MAX_TL_ADD_SIZE}')
            logging.info(f'MAX_TL_DROP_SIZE: {cfg.TABU_MAX_TL_DROP_SIZE}')
            logging.info(f'MAX_TL_REDISTRIBUTE_SIZE: {cfg.TABU_TL_REDISTRIBUTE_SIZE}')
            logging.info(f'MAX_ITER: {cfg.TABU_MAX_ITER}')
            logging.info(f'FIND_FROM_ELITE_AFTER: {cfg.TABU_FIND_FROM_ELITE_AFTER}')
            logging.info(f'RESTART_AFTER: {cfg.TABU_RESTART_AFTER}')
            logging.info(f'REROUTE_ON_CRITICAL: {cfg.TABU_REROUTE_ON_CRITICAL}')
            logging.info(f'REVERSE_ON_CRITICAL: {cfg.TABU_REVERSE_ON_CRITICAL}')
            logging.info(f'REVERSE_CRITICAL_LONGEST_EDGE: {cfg.TABU_REROUTE_LONGEST_EDGE}')
            
    def check_weight_zero(self, graph):
        for a,b,info in graph.edges(data = True):
            if info['weight'] == 0:
                logging.info(f'Edge {a,b} has weight 0')

    def save_fitness_progress(self, data_name):
        # Save the best fitness progress to a txt file
        cfg_info = [f'RANDOM_SEED: {cfg.RND_SEED}',
                    f'NUM_REVERSE_NEIGHBORS: {cfg.TABU_NUM_REVERSE_NEIGHBORS}',
                    f'NUM_REROUTE_NEIGHBORS: {cfg.TABU_NUM_REROUTE_NEIGHBORS}',
                    f'NUM_REDISTRIBUTE_NEIGHBORS: {cfg.TABU_NUM_REDISTRIBUTE_NEIGHBORS}',
                    f'MAX_TL_ADD_SIZE: {cfg.TABU_MAX_TL_ADD_SIZE}',
                    f'MAX_TL_DROP_SIZE: {cfg.TABU_MAX_TL_DROP_SIZE}',
                    f'MAX_TL_REDISTRIBUTE_SIZE: {cfg.TABU_TL_REDISTRIBUTE_SIZE}',
                    f'MAX_ITER: {cfg.TABU_MAX_ITER}',
                    f'FIND_FROM_ELITE_AFTER: {cfg.TABU_FIND_FROM_ELITE_AFTER}',
                    f'RESTART_AFTER: {cfg.TABU_RESTART_AFTER}',
                    f'REROUTE_ON_CRITICAL: {cfg.TABU_REROUTE_ON_CRITICAL}',
                    f'REVERSE_ON_CRITICAL: {cfg.TABU_REVERSE_ON_CRITICAL}',
                    f'REVERSE_CRITICAL_LONGEST_EDGE: {cfg.TABU_REROUTE_LONGEST_EDGE}',
                    f'cfg.TABU_REDISTRIBUTE_PROB: {cfg.TABU_REDISTRIBUTE_PROB}',
                    "SOLVE_RESULT"
                    ]
        
        best_fitnesses_with_cfg = cfg_info + self.best_fitnesses
        cur_fitnesses_with_cfg = cfg_info + self.cur_fitnesses
        best_fitness_file_name = f'{data_name}_best_fitnesses'
        cur_fitnesses_file_name = f'{data_name}_cur_fitnesses'
        i = 0
        while os.path.exists(f'solve_results/tabu/best_fitnesses/{best_fitness_file_name}{i}.txt'):
            i += 1
        np.savetxt(f'solve_results/tabu/best_fitnesses/{best_fitness_file_name}{i}.txt', best_fitnesses_with_cfg,fmt='%s')
        np.savetxt(f'solve_results/tabu/cur_fitnesses/{cur_fitnesses_file_name}{i}.txt', cur_fitnesses_with_cfg,fmt='%s')

    def search(self, total_robots, max_iters, tabu_list_size =cfg.TABU_MAX_TL_ADD_SIZE):
        self._log_config()

        init_sol = self._init_solution(total_robots)
        if type(init_sol) != nx.DiGraph:
            init_sol_makespan = init_sol.get_makespan()
            logging.info(f"Initial makespan: {init_sol_makespan}")
            idx_paths = init_sol.get_idx_paths(change_sink_idx = True)
            G = self._to_graph(idx_paths)

        elif type(init_sol) == nx.DiGraph:
            G = init_sol

        best_solution = G
        lastest_best_solution = G
        best_solution_fitness = self._get_fitness(best_solution)
        logging.info(f'Initial fitness: {best_solution_fitness}')
        tl_add = []
        tl_drop = []
        tl_redistribute = []
        i_no_update = 0
        best_fitnesses = [best_solution_fitness]
        cur_fitnesses = [best_solution_fitness]

        num_improve = {0:0, 1:0} # 0 is for reroute or reverse, 1 is for redistribute, number of times the best solution is improved by each neighborhood
        total_improve = {0:0, 1:0} # total improve of fitness by each neighborhood, used to calculate the average improve of each neighborhood when it improves the best solution

        for i in range(max_iters):
            # logging.info(f'Iteration {i} started')
            affected_add = {} #key : graph, value:edges to add to tl_add
            affected_drop = {} #key : graph, value:edges to add to tl_drop
            affected_redistribute = {}
            neighbors = self._get_neighbors(lastest_best_solution, affected_add, affected_drop, affected_redistribute)
            best_neighbor = None
            best_neighbor_0 = None # save best neighbor in reroute or reverse neighborhood
            best_neighbor_fitness = np.inf
            best_neighbor_fitness_0 = np.inf
            chosen_neigborhood = 0 # 0 is for reroute or reverse, 1 is for redistribute
            for neighbor in neighbors:
                if neighbor in affected_add: #reroute or reverse
                    if  not self._is_tabu(affected_add[neighbor], affected_drop[neighbor],tl_add, tl_drop): #TODO find a way to check this quickly
                        neighbor_fitness = self._get_fitness(neighbor)
                        if best_neighbor is None or neighbor_fitness < best_neighbor_fitness:
                            chosen_neigborhood = 0
                            best_neighbor = neighbor
                            best_neighbor_fitness = neighbor_fitness
                        if best_neighbor_0 is None or neighbor_fitness < best_neighbor_fitness_0:
                            best_neighbor_0 = neighbor
                            best_neighbor_fitness_0 = neighbor_fitness
                if neighbor in affected_redistribute:
                    if  not self._is_tabu_redistribute(affected_redistribute[neighbor], tl_redistribute):
                        neighbor_fitness = self._get_fitness(neighbor)
                        # print(f'Neighbor fitness redistribute: {neighbor_fitness}') #TODO delete
                        if best_neighbor is None or neighbor_fitness < best_neighbor_fitness:
                            chosen_neigborhood = 1
                            best_neighbor = neighbor
                            best_neighbor_fitness = neighbor_fitness
            
            # logging.info(f'Chosen neighborhood: {chosen_neigborhood}')
            if best_neighbor is None:
                logging.info(f'No neighbor found')
                break

            if chosen_neigborhood == 1 and np.abs(best_neighbor_fitness - cur_fitnesses[-1]) < 0.0001:
                #beta probability to choose redistribute, 1 - beta to choose reroute or reverse
                beta = np.random.rand()
                if beta > cfg.TABU_REDISTRIBUTE_PROB and affected_add:
                    chosen_neigborhood = 0
                    best_neighbor = best_neighbor_0
                    best_neighbor_fitness = best_neighbor_fitness_0
                    # logging.info("Changed to best reroute or reverse neighbor")

            if best_neighbor_fitness < best_solution_fitness:
                num_improve[chosen_neigborhood] += 1
                total_improve[chosen_neigborhood] += best_solution_fitness - best_neighbor_fitness

                best_solution = best_neighbor
                best_solution_fitness = best_neighbor_fitness
                logging.info(f'Best fitness updated: {best_solution_fitness}, chosen neighborhood: {chosen_neigborhood}')
                i_no_update = 0
            else:
                i_no_update += 1


            lastest_best_solution = best_neighbor
            if chosen_neigborhood == 0:
                tl_add += affected_add[best_neighbor]
                tl_drop += affected_drop[best_neighbor]
            if chosen_neigborhood ==1:
                tl_redistribute += affected_redistribute[best_neighbor]
            if len(tl_add) > tabu_list_size:
                # logging.info(f"Removing {len(tl_add) - tabu_list_size} item from tb list")
                [tl_add.pop(i) for i in range(len(tl_add) - tabu_list_size)]
            if len(tl_drop) > tabu_list_size:
                [tl_drop.pop(i) for i in range(len(tl_drop) - tabu_list_size)]
            if len(tl_redistribute) > cfg.TABU_TL_REDISTRIBUTE_SIZE:
                [tl_redistribute.pop(i) for i in range(len(tl_redistribute) - cfg.TABU_TL_REDISTRIBUTE_SIZE)]
            if i % cfg.TABU_LOG_EVERY_ITERATIONS == 0:
                logging.info(f'Iteration {i} done')


            if i_no_update == cfg.TABU_FIND_FROM_ELITE_AFTER:
                lastest_best_solution = best_solution
                logging.info(f'No update for {cfg.TABU_FIND_FROM_ELITE_AFTER} iterations, re-search from best solution')
                # i_no_update = 0
            
            if i_no_update == cfg.TABU_RESTART_AFTER:
                logging.info(f'No update for {cfg.TABU_RESTART_AFTER} iterations, restarting from initial solution')
                cur_best_makespan = np.infty
                for i in range(cfg.VIET_RANDOM_ITE):
                    solve_viet = VGeneralizedSolver(self.processed, self.c_travel, self.c_operate)
                    sol = solve_viet.solve(total_robots,'random',cfg.TABU_INIT_CONSIDER_PROCESS_TIME,task_range=[1,3])
                    makespan = sol.get_makespan()
                    idx_paths = sol.get_idx_paths(change_sink_idx = True)
                    G = self._to_graph(idx_paths)
                    if makespan < cur_best_makespan:
                        cur_best  = G
                        cur_best_makespan = makespan
                
                if cur_best_makespan < best_solution_fitness:
                    best_solution = cur_best
                    best_solution_fitness = cur_best_makespan
                    
                lastest_best_solution = G
                i_no_update = 0
            
            best_fitnesses.append(best_solution_fitness)
            cur_fitnesses.append(best_neighbor_fitness)
        
        self.best_fitnesses = best_fitnesses
        self.cur_fitnesses = cur_fitnesses
        avg_improve = {0: total_improve[0]/num_improve[0] if num_improve[0]>0 else 0, 1: total_improve[1]/num_improve[1] if num_improve[1]>0 else 0}
        logging.info(f'Best solution found: {best_solution_fitness}')
        return best_solution, best_solution_fitness, num_improve, avg_improve
    

class GreedyInsert:
    """
    Construct a solution from scratch using greedy insert heuristic.
    In this version, we can insert task anywhere on the flow DAG.
    Based on the idea of Redistribute neighborhood.
    """

    def __init__(self, processed, c_travel, c_operate) -> None:
        self.processed = processed
        self.c_travel = c_travel
        self.c_operate = c_operate

    def _init_F_(self,num_robots):
        """
        Initialize the Flow DAG, it contains the source, sink and there are num_robots 
        flows from source to sink.

        """
        F_cur = nx.DiGraph()
        F_cur.add_node(0)
        F_cur.add_node(-1)
        F_cur.add_weighted_edges_from([(0,-1,num_robots)])
        return F_cur


    def _insert(self, graph: nx.DiGraph, topo_order :list, cut_point :int, Ls, i_hat, n_i_hat, insert_algo = 2):
        """Alain's idea of cut and redistribute
        
        Parameters
        ----------
        graph : nx.DiGraph
            graph representation of the solution
        topo_order : _type_
            _description_
        cut_point : _type_
            _description_
        insert_algo:int
            1: Cplex insert, 2: dedicated insert
        """
        #cut
        #calculate pi_i left and right
        N_left = topo_order[:cut_point]
        N_right = topo_order[cut_point:]
        pi_left = {node: np.sum(graph.edges[node,succ]['weight'] for succ in graph.successors(node) if succ in N_right) for node in N_left}
        pi_right = {node: np.sum(graph.edges[pred,node]['weight'] for pred in graph.predecessors(node) if pred in N_left) for node in N_right}
        result_graph = deep_copy_graph(graph)
        edges = list(graph.edges())

        if insert_algo ==1: #CPLEX insert
            logging.info(f'Cplex insert task {i_hat}')
            for edge in edges:
                if edge[0] in N_left and edge[1] in N_right:
                    result_graph.remove_edge(edge[0], edge[1])
            solve_results = self._cplex_insert(N_left, N_right, pi_left, pi_right, Ls, i_hat,n_i_hat)
            f = solve_results['sol'].get_value_dict(solve_results['f'], keep_zeros = False)
            C_max = solve_results['sol'].get_objective_value()
            for key, value in f.items():
                result_graph.add_edge(key[0],key[1], weight = value)
            # if cfg.GREEDY_DEBUG:
            #     new_topo_order = list(nx.topological_sort(result_graph))
            #     C_max_real,*rest = self._get_related_times(result_graph,self.c_travel,self.c_operate,new_topo_order)
            #     if C_max_real[-1]!= C_max:
            #         print(f'C_max real: {C_max_real[-1]}, C_max: {C_max}')
                # assert C_max_real[-1] == C_max
                    
        else: #dedi insert
            logging.info(f'Dedicated insert task {i_hat}')
            result_graph, C_max = self._high_dedicated_insert(result_graph, N_left, N_right, pi_left, pi_right, Ls, i_hat, n_i_hat)
            if cfg.GREEDY_DEBUG:
                new_topo_order = list(nx.topological_sort(result_graph))
                C_max_real,*rest = self._get_related_times(result_graph,self.c_travel,self.c_operate,new_topo_order)
                if not helpers.approx_equal(C_max_real[-1],C_max):
                # if C_max_real[-1]!= C_max:
                    print(f'C_max real: {C_max_real[-1]}, C_max: {C_max}')
            # logging.info(f'C_max dedicated after insert task {i_hat}: {C_max}')
        return result_graph, C_max
    
    def _cplex_insert(self, N_left, N_right, pi_left, pi_right, Ls, i_hat,n_i_hat):
        L, L_bar, L_star,_ = Ls
        c_travel = self.c_travel
        c_operate = self.c_operate

        m = Model('greedy2')
        N_left_i_hat = N_left + [i_hat]
        N_right_i_hat = N_right + [i_hat]
        N_index = [(i,j) for i in N_left_i_hat for j in N_right_i_hat]
        x = m.binary_var_dict(N_index, name = 'x')
        f = m.continuous_var_dict(N_index, name = 'f')
        C_max = m.continuous_var(name = 'C_max')
        c_max = m.continuous_var(name = 'c_max')

        m.minimize(C_max)
        m.add_constraints((L_bar[i] + c_travel[i][j] + L_star[j])*x[i,j] <= C_max for i in N_left for j in N_right)
        m.add_constraints((L_bar[i] +c_travel[i][i_hat] + c_operate[i_hat])*x[i,i_hat] <= c_max for i in N_left)
        m.add_constraints(c_max + (c_travel[i_hat][j] + L_star[j])*x[i_hat,j] <= C_max for j in N_right)
        m.add_constraints(m.sum(f[i,j] for j in N_right_i_hat) == pi_left[i] for i in N_left)
        m.add_constraints(m.sum(f[i,j] for i in N_left_i_hat) == pi_right[j] for j in N_right)
        m.add_constraint(m.sum(f[i,i_hat] for i in N_left) == n_i_hat)
        m.add_constraint(m.sum(f[i_hat,j] for j in N_right) == n_i_hat)
        m.add_constraints(f[i,j] <= 30*x[i,j] for i in N_left_i_hat for j in N_right_i_hat)
        m.add_constraints(f[i,j] >= x[i,j] for i in N_left_i_hat for j in N_right_i_hat)
        
        cplex_sol = m.solve(log_output = cfg.LOG_OUTPUT)
        # print(cplex_sol.objective_value) #TODO delete
        if cfg.EXPORT_MODEL == True:
            print(m.export_as_lp())
        solved_gap = m.solve_details.mip_relative_gap
        processed_nodes = m.solve_details.nb_nodes_processed
        solve_time = m.solve_details.time

        solve_info = {
            'sol':cplex_sol,
            'x':x,
            'f':f,
            "C_max":C_max,
            'processed_nodes': processed_nodes,
            'gap':solved_gap,
            'solve_time':solve_time
        }
        return solve_info

    def _high_dedicated_insert(self, sol_graph, N_left, N_right, pi_left, pi_right, Ls, i_hat, n_i_hat):
        """
        G2P High level
        """
        #Init
        if cfg.GREEDY_DEBUG:
            # logging.info('start_high_dedicated_insert')
            tasks = [task for task in self.processed.tasks]
            helpers._check_graph_feasible(sol_graph,tasks)
        i_hat_l = 999999
        i_hat_r = i_hat_l +1
        for dest in self.c_travel:
            if dest != -1:
                self.c_travel[dest][i_hat_r] = self.c_travel[dest][i_hat]
        self.c_travel[i_hat_l] = self.c_travel[i_hat]
        N_left_i_hat = N_left + [i_hat_l]
        N_right_i_hat = N_right + [i_hat_r]
        L, L_bar, L_star,_ = Ls
        L_star[i_hat_r] = 0

        thetas = [L_bar[i] + self.c_travel[i][i_hat] + self.c_operate[i_hat] for i in N_left]
        thetas.sort()
        L_bar[i_hat_l] = thetas.pop()
        sol_graph,C_max = self._init_sol_g2p_high(sol_graph, N_left, N_right, pi_left, pi_right,Ls,i_hat, i_hat_l, i_hat_r, n_i_hat)
        # if cfg.GREEDY_DEBUG:
        #     tasks = [task for task in self.processed.tasks]
        #     helpers._check_graph_feasible(sol_graph,tasks)

        pi_left[i_hat_l] = n_i_hat
        pi_right[i_hat_r] = n_i_hat
        forbid_nodes = [i_hat_l]
        sol_graph,C_max, Stop = self._low_dedicated_insert(
            sol_graph, C_max, N_left_i_hat, N_right_i_hat, pi_left, pi_right, Ls, i_hat, i_hat_l, i_hat_r, forbid_nodes)
        Stop = False
        while (Stop is False) and (thetas):
            # logging.info(f'Current thetas: {thetas}')
            L_bar_i_hat = thetas.pop()
            L_bar[i_hat_l] = L_bar_i_hat
            forbid_nodes = self._construct_forbid_n_left(N_left, i_hat, i_hat_l, Ls, L_bar_i_hat)
            sol_graph ,C_max, Stop =  self._low_dedicated_insert(
                sol_graph,C_max, N_left_i_hat, N_right_i_hat, pi_left, pi_right, Ls, i_hat,i_hat_l, i_hat_r, forbid_nodes)
            if cfg.GREEDY_DEBUG:
                #2798.716951071826
                if helpers.approx_equal(C_max, 2798.71695):
                    print('Something wrong in high')
        #replace artifical tasks with real task
        sol_graph = self._replace_clone_tasks(sol_graph, i_hat, i_hat_l, i_hat_r)
        return sol_graph, C_max
    
    def _replace_clone_tasks(self, graph_with_clone: nx.DiGraph, i_hat: int, i_hat_l: int, i_hat_r: int):
        """
        Replace the nodes i_hat_l, i_hat_r with i_hat in the graph
        """
        #Replace the nodes i_hat_l, i_hat_r with i_hat in the graph, and update the weights of the edges
        graph = deep_copy_graph(graph_with_clone)
        edges = list(graph.edges())
        for edge in edges:
            if edge[0] == i_hat_l:
                edge_weight = graph.edges[edge]['weight']
                graph.add_edge(i_hat, edge[1], weight = edge_weight)
                # graph.remove_edge(i_hat_l, edge[1])
            if edge[1] == i_hat_r:
                edge_weight = graph.edges[edge]['weight']
                graph.add_edge(edge[0], i_hat, weight = edge_weight)
                # graph.remove_edge(edge[0], i_hat_r)
        graph.remove_nodes_from([i_hat_l,i_hat_r])
        
        return graph
            

    def _init_sol_g2p_high(self, sol_graph, N_left, N_right, pi_left, pi_right, Ls, i_hat, i_hat_l, i_hat_r, n_i_hat):
        """insert current node i to the graph
        by letting n_i_hat flows come from N_left to i_hat
        and letting n_i_hat flows go from i_hat to N_right
        """
        L, L_bar, L_star,_ = Ls
        f = self._build_f(sol_graph, N_left, N_right, pi_left, pi_right) #TODO maybe unnessessary
        n_remain = n_i_hat
        for edge, f_value in f.items():
            if f_value <= n_remain: # Current edge cannot cover enough flows for the task
                i,j = edge
                n_remain = n_remain - f_value
                # f.pop(edge)
                #Check if the edge is in the graph:
                if sol_graph.has_edge(i,i_hat_r):
                    sol_graph[i][i_hat_r]['weight'] += f_value
                else:
                    sol_graph.add_edge(i,i_hat_r, weight = f_value)
                
                if sol_graph.has_edge(i_hat_l,j):
                    sol_graph[i_hat_l][j]['weight'] += f_value
                else:
                    sol_graph.add_edge(i_hat_l, j, weight = f_value)
                sol_graph.remove_edge(i,j)
            else: # Current edge can cover enough flows for the task
                i,j = edge
                # f[edge] = f_value - n_remain
                if n_remain !=0:
                    if sol_graph.has_edge(i,i_hat_r):
                        sol_graph[i][i_hat_r]['weight'] += n_remain
                    else:
                        sol_graph.add_edge(i,i_hat_r, weight = n_remain)
                    
                    if sol_graph.has_edge(i_hat_l,j):
                        sol_graph[i_hat_l][j]['weight'] += n_remain
                    else:
                        sol_graph.add_edge(i_hat_l, j, weight = n_remain)
                #reduce weight on edge (i,j) of sol_graph by n_remain
                sol_graph[i][j]['weight'] = sol_graph[i][j]['weight'] - n_remain
                if sol_graph[i][j]['weight'] == 0:
                    sol_graph.remove_edge(i,j)
                n_remain = 0
                break
        
        #Calculate real C_max
        N_left_i_hat= N_left + [i_hat_l]
        N_right_i_hat = N_right + [i_hat_r]
        pi_right[i_hat_r] = n_i_hat
        pi_left[i_hat_l] = n_i_hat
        f = self._build_f(sol_graph, N_left_i_hat, N_right_i_hat, pi_left, pi_right)
        try:
            C_max_non_i_star = max([L_bar[i] + self.c_travel[i][j] + L_star[j] for (i,j) in f if i != i_hat_l if j != i_hat_r])
        except ValueError:
            C_max_non_i_star = 0
        real_L_star_i_hat = max([L_bar[i] + self.c_travel[i][i_hat] + self.c_operate[i_hat] for i in N_left])
        C_max_i_star = real_L_star_i_hat + max([self.c_travel[i_hat][j] + L_star[j] for j in N_right if (i_hat_l,j) in f])
        C_max = max(C_max_non_i_star, C_max_i_star)

        return sol_graph, C_max
            
    def _construct_forbid_n_left(self, N_left, i_hat, i_hat_l, Ls, L_bar_i_hat):
        """Construct a set of node i such that (i,i_hat_r)
        are forbidden in G2P algorithm

        Parameters
        ----------
        N_left : _type_
            _description_
        i_hat : _type_
            _description_
        Ls : _type_
            _description_
        L_bar_i_hat : _type_
            _description_

        Returns
        -------
        _type_
            _description_
        """
        forbid_nodes =  [i_hat_l]   
        L, L_bar, L_star,_ = Ls
        for i in N_left:
            if L_bar[i] + self.c_travel[i][i_hat] + self.c_operate[i_hat] > L_bar_i_hat:
                forbid_nodes.append(i)
        return forbid_nodes

    def _low_dedicated_insert(self,sol_graph, C_max, N_left, N_right, pi_left, pi_right, Ls, i_hat, i_hat_l, i_hat_r, forbid_nodes):
        """
        G2P low level (or "IMPROVE prcedure" in Alain's idea)
        """
        # N_left_i_hat = N_left + [i_hat_l]
        # N_right_i_hat = N_right + [i_hat_r]
        _, l_bar, l_star, _ = Ls
        sol_graph = deep_copy_graph(sol_graph)
        deltas = {
            (i,j): l_bar[i] + self.c_travel[i][j] + l_star[j] for i in N_left for j in N_right
            }
        # f = self._build_f(sol_graph, N_left, N_right, pi_left, pi_right)
        # C_max = max(deltas[(i,j)] for i,j in f)
        Af= [(i,j) for i in N_left for j in N_right  if pi_left[i] > 0 and pi_right[j] > 0 and deltas[(i,j)] > C_max] #TODO check if >= C_max or > C_max
        fail_feasible = False
        success_feasible = False
        forbid_arcs = [(i,i_hat_r) for i in forbid_nodes]
        while (fail_feasible is False) and (success_feasible is False):
            f = self._build_f(sol_graph, N_left, N_right, pi_left, pi_right)
            forbid_has_flows = [i for i in forbid_nodes if (i,i_hat_r) in f] #TODO is i_hat_r in sol graph
            if not forbid_has_flows:
                success_feasible = True
                break
            forbid_gamma = Af + forbid_arcs
            i_star = forbid_has_flows.pop()
            gamma = self._find_gamma(
                N_left, N_right, forbid_gamma, f, i_star, i_hat_r)
            if gamma is None:
                fail_feasible = True
                break
            else:
                try:
                    delta_f = self._redis_cal_delta_flows(gamma, f)
                    sol_graph = self._redis_update_graph(sol_graph, gamma, delta_f)
                    # Update C_max
                    new_C_max = max(deltas[(i,j)] for i,j in f)
                    if new_C_max < C_max:
                        C_max = new_C_max
                except Exception as e:
                    logging.error(f'Error in low dedicated insert:')
                    raise e
        
        if fail_feasible:
            return sol_graph, C_max, fail_feasible
        if success_feasible:
            sol_graph,   = self._dedicated_redistribute(
                sol_graph, C_max, N_left, N_right, pi_left, pi_right, Ls, forbid_arcs)
            if cfg.GREEDY_DEBUG:
                #2798.716951071826
                if helpers.approx_equal(C_max, 2798.71695):
                    print('Something wrong in lowww successs')
            return sol_graph, C_max, fail_feasible

    def _redis_cal_delta_flows(self, gamma, f) -> int:
        """Generate maximum number of flows to be redistributed

        Parameters
        ----------
        gamma : list[int]
            The path from i_star to j_star
        f : dict
            _description_

        Returns
        -------
        int
            number of flows to be redistributed
        """
        affected_edges = [(gamma[2*k], gamma[2*k - 1]) for k in range(1, len(gamma)//2)]
        affected_flows = []
        for edge in affected_edges:
            affected_flows.append(f[edge])
        longest_edge = f[(gamma[0], gamma[-1])]
        affected_flows.append(longest_edge)

        return min(affected_flows)

    def _find_gamma(self, N_left, N_right, Af, f, i_star, j_star):
        
        queue = deque([([i_star], i_star, None)])  # Start with i_star, no j yet
        visited = set()  # To avoid revisiting the same i, j combinations

        while queue:
            path, current_i, current_j = queue.popleft()

            # If the last node in the path is j_star, return the found path
            if current_j == j_star:
                return path

            if current_j is None:
                # Initial step: find j_1 such that (i_star, j_1) is not in Af
                for j in N_right:
                    if (i_star, j) not in Af:
                        new_path = path + [j]  # Add j to path
                        queue.append((new_path, i_star, j))
                        visited.add((i_star, j))
            else:
                # Intermediate steps: find valid (i_k, j_k) pairs
                for i_next in N_left:
                    if (i_next, current_j) in f and (i_next, current_j) not in Af:
                        for j_next in N_right:
                            if (i_next, j_next) not in Af and (i_next, j_next) not in visited:
                                new_path = path + [i_next, j_next]  # Add both i and j to path
                                queue.append((new_path, i_next, j_next))
                                visited.add((i_next, j_next))

        # If no valid path is found
        
        return None


    def _build_f(self, sol_graph, N_left, N_right, pi_left, pi_right):
        """
        Return flow value between every pair in N_left and N_right
        """
        f = {}
        for i in N_left:
            if pi_left[i] > 0:
                for j in N_right:
                    if pi_right[j] > 0:
                        try:
                            f[i,j] = sol_graph[i][j]['weight']
                        except:
                            pass
        return f
    
    def _dedicated_redistribute(self, sol_graph,C_max, N_left, N_right, pi_left, pi_right, Ls, forbid_arcs):
        """
        Generate a new solution by redistributing the tasks in N_left to N_right
        using Alain's idea
        """
        l, l_bar, l_star,_ = Ls
        sol_graph = deep_copy_graph(sol_graph)
        deltas = {
            (i,j): l_bar[i] + self.c_travel[i][j] + l_star[j] for i in N_left for j in N_right
            }
        improved = False
        stop = False
        while not stop:
            # f = {(i,j): sol_graph[i][j]['weight'] for i in N_left for j in N_right if pi_left[i] > 0 and pi_right[j] > 0}
            pi_left = {node: np.sum(sol_graph.edges[node,succ]['weight'] for succ in sol_graph.successors(node) if succ in N_right) for node in N_left}
            pi_right = {node: np.sum(sol_graph.edges[pred,node]['weight'] for pred in sol_graph.predecessors(node) if pred in N_left) for node in N_right}
            f = self._build_f(sol_graph, N_left, N_right, pi_left, pi_right)
            C_max = min(C_max,max(deltas[(i,j)] for i in N_left for j in N_right if (i,j) in f))
            
            if cfg.GREEDY_DEBUG:
                if helpers.approx_equal(C_max, 2798.71695):
                    print('C_max: ', C_max)
                    print(max(deltas[(i,j)] for i in N_left for j in N_right if (i,j) in f))
                    print('Something wrong in dedicated none')

            Af= [(i,j) for i in N_left for j in N_right  if pi_left[i] > 0 and pi_right[j] > 0 and deltas[(i,j)] >= C_max]
            A_max = [edge for edge in Af if edge in f]
            forbidden_gamma = Af + forbid_arcs

            if len(A_max) > 0:
                selected_edge = random.choice(A_max)
            (i_star,j_star) = selected_edge

            gamma = self._find_gamma(N_left, N_right, forbidden_gamma, f, i_star, j_star)
            if gamma == None:

                if cfg.GREEDY_DEBUG:
                    #2798.716951071826
                    if helpers.approx_equal(C_max, 2798.71695):
                        print('Something wrong in gamma none')

                stop = True
            else:
                # Update f
                delta_f = self._redis_cal_delta_flows(gamma, f)
                sol_graph = self._redis_update_graph(sol_graph, gamma, delta_f)
                # Update C_max
                new_C_max = max(deltas[(i,j)] for i,j in f)
                if new_C_max < C_max:
                    C_max = new_C_max

                    if cfg.GREEDY_DEBUG:
                        #2798.716951071826
                        if helpers.approx_equal(C_max, 2798.71695):
                            print('Something wrong gamma right')

                    improved = True
        return sol_graph, C_max

    def _redis_update_graph(self, graph, gamma, delta_f):
        result_graph = deep_copy_graph(graph)
        decrease_edges = [(gamma[2*k], gamma[2*k - 1]) for k in range(1, len(gamma)//2)]
        decrease_edges.append((gamma[0], gamma[-1]))
        increase_edges = [(gamma[2*k], gamma[2*k + 1]) for k in range(1, len(gamma)//2 - 1)] #TODO check off by 1
        increase_edges += [(gamma[0], gamma[1]), (gamma[-2], gamma[-1])] # (i_star,j_1) and (i_{K},j_star)

        for edge in decrease_edges:
            result_graph[edge[0]][edge[1]]['weight'] -= delta_f
            if result_graph[edge[0]][edge[1]]['weight'] == 0:
                result_graph.remove_edge(edge[0], edge[1])
        
        for edge in increase_edges:
            if result_graph.has_edge(edge[0], edge[1]):
                result_graph[edge[0]][edge[1]]['weight'] += delta_f
            else:
                result_graph.add_edge(edge[0], edge[1], weight = delta_f)

        return result_graph
    def _get_out_nodes(self,graph):
        """
        Get a dictionary of out node from all nodes on a directed graph
        
        Parameters
        ----------
        graph : nx.DiGraph or nx.MultiDiGraph


        Returns
        -------
        out_nodes : dict
            `out_nodes[1] = [2,3,69]` means that edges (1,2), (1,3), (1,69) in the graph.
        """
        out_nodes = {}
        nodes = list(graph.nodes())
        for node in nodes:
            out_nodes[node] = list(graph.successors(node))

        return out_nodes

    def _get_in_nodes(self,graph):
        """
        Get a dictionary of in nodes from all nodes on a directed graph
        
        Parameters
        ----------
        graph : nx.DiGraph or nx.MultiDiGraph


        Returns
        -------
        in_nodes : dict
            `in_nodes[35] = [2,3,69]` means that edges (2,35), (3,35), (69,35) are in the graph.
        """
        in_nodes = {}
        nodes = list(graph.nodes())
        for node in nodes:
            in_nodes[node] = list(graph.predecessors(node))

        return in_nodes
    

    def _get_longest_path(self, graph, c_travel, c_operate, topo_order):
        """Get longest path from each node to the sink node (or Lstar in redistribute neighborhood)
        Parameters
        ----------
        graph : _type_
            _description_
        c_travel : _type_
            _description_
        c_operate : _type_
            _description_

        Returns
        -------
        _type_
            _description_
        """
        topo_order = copy.deepcopy(topo_order)
        # topo_order = list(nx.topological_sort(graph))
        in_nodes = self._get_in_nodes(graph)
        # c_travel = copy.deepcopy(c_travel)
        # c_operate = copy.deepcopy(c_operate)
        
        c_operate[-1] = 0
        c_travel[-1] = {}
        c_travel[0][-1] = 0
        for i in c_travel:
            if i!= 0 and i!=-1:
                c_travel[i][-1] = c_travel[i][0]
        
        L_star = {}
        for j in c_operate:
            L_star[j] = -np.infty
        L_star[-1] = 0
        L_star[0] = -np.infty
        
        while topo_order:
            cur_task = topo_order.pop()
            for pred in in_nodes[cur_task]:
                cur_L_star = L_star[cur_task] + c_operate[pred] + c_travel[pred][cur_task]
                if L_star[pred] < cur_L_star:
                    L_star[pred] = cur_L_star

        return L_star

    def _get_related_times(self, graph, c_travel, c_operate, topo_order):
        """
        Calculate the L, L_bar, L_star and waiting time of the graph,
        see greedy alain for details:
            L[j]: the earliest time of task j can start doing
            L_bar[j]: the earliest time of task j can finish
            L_star[j]: the time of a longest path from pickup point of j to the source.
        Parameters
        ----------
        graph : _type_
            _description_
        c_travel : _type_
            _description_
        c_operate : _type_
            _description_

        Returns
        -------
        _type_
            _description_
        """
        reversed_topo_order = list(reversed(topo_order))
        out_nodes = self._get_out_nodes(graph)
        # c_travel = copy.deepcopy(c_travel)
        # c_operate = copy.deepcopy(c_operate)
        c_operate[-1] = 0
        c_travel[-1] = {}
        c_travel[0][-1] = 0
        for i in c_travel:
            if i!= 0 and i!=-1:
                c_travel[i][-1] = c_travel[i][0]
        
        z = {} #z[i] is the start time of task i
        for j in c_operate:
            z[j] = -np.infty
        z[0] = 0
        z[-1] = -np.infty
        while reversed_topo_order:
            cur_task = reversed_topo_order.pop()
            for succ in out_nodes[cur_task]:
                z_cur = z[cur_task] + c_operate[cur_task] + c_travel[cur_task][succ]
                if z[succ] < z_cur:
                    z[succ] = z_cur
        
        edges = list(graph.edges())
        waits = {} # w[(a,b)] is the waitng time of robots start from taks a to task b on task b
        for edge in edges:
            (a,b) = edge
            waits[edge] = z[b] - z[a] - c_operate[a] - c_travel[a][b]
        
        L = z
        L_bar = {node: L[node] + self.c_operate[node] for node in L}
        L_star = self._get_longest_path(graph, self.c_travel, self.c_operate, topo_order)
        return L, L_bar, L_star, waits
    
    def solve(self, num_robots,insert_algo= 2, insert_sequence = None):
        """_summary_

        Parameters
        ----------
        num_robots : _type_
            _description_
        insert_algo : int, optional
            _description_, by default 2
        insert_sequence : list, optional
            list of algoirthm to insert task at each iteration, by default None

        Returns
        -------
        _type_
            _description_
        """
        # if cfg.DEDICATED_INSERT:
        #     logging.info(f'Dedicated insert')
        # else:
        #     logging.info(f'CPLEX insert')
        tasks = self.processed.tasks
        depot = self.processed.depot
        idx_task_mapper = {task.index: task for task in tasks}
        U = [task.index for task in tasks]
        zeta_max = 2
        F_cur = self._init_F_(num_robots)
        F_best = deep_copy_graph(F_cur)

        F_history = []
        C_max_history = []
        while U:
            logging.info(f'zeta_max: {zeta_max}')
            C_max_best = np.infty
            i_hat = np.random.choice(U) # i_hat : chosen task
            if cfg.DEBUG_IHAT_NOT_RANDOM:
                i_hat = U[0] #TODO DELETE AFTER FINISH TESTING!!
            n_i_hat = idx_task_mapper[i_hat].n
            U.remove(i_hat)
            topo_order = list(nx.topological_sort(F_cur))
            if insert_sequence:
                insert_algo = insert_sequence.pop(0)
            for zeta in range(1, zeta_max):
                # logging.info(f'zeta: {zeta}')
                Ls = self._get_related_times(F_cur, self.c_travel, self.c_operate, topo_order)
                F_cur_zeta, C_max_cur = self._insert(F_cur, topo_order, zeta, Ls, i_hat, n_i_hat, insert_algo)
                # logging.info(f'current edges: {F_cur_zeta.edges()}')
                # logging.info(f'C_max: {C_max_cur}')
                if C_max_cur < C_max_best:
                    F_best = deep_copy_graph(F_cur_zeta)
                    C_max_best = C_max_cur
                    if cfg.GREEDY_DEBUG:
                        logging.info(f'New best C_max: {C_max_best}')
            F_cur = F_best
            if cfg.GREEDY_DEBUG:
                tasks = [task for task in self.processed.tasks]
                helpers._check_graph_feasible(F_cur,tasks)
                F_history.append(deep_copy_graph(F_cur)) #TODO remove after testing
                C_max_history.append(C_max_best)
            zeta_max += 1
            # logging.info(f'current edges: {F_cur.edges()}')
        return F_best, C_max_best, F_history, C_max_history
    

class SolutionFetcher:
    def __init__(self, data_path):
        self.data_path = data_path

    def fetch(self):
        solution = pickle.load(open(f'{self.data_path}.p', 'rb'))
        return solution
    

def deep_copy_graph(graph):
    new_graph = nx.DiGraph()
    # new_graph.add_nodes_from(graph.nodes())
    new_graph.add_edges_from(graph.edges(data = True))
    return new_graph
        