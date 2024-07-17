import copy
from docplex.mp.model import Model
import heapq
import networkx as nx
import numpy as np
import pandas as pd
import time 
import os

import config as cfg
import helpers
import logging
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


class TabuSearch:
    def __init__(self, processed, c_travel, c_operate, init_solution_func = VGeneralizedSolver):
        self.processed = processed
        self.c_travel = c_travel
        self.c_operate = c_operate
        self.init_solution_func = init_solution_func

    def set_init_func(self, init_solution_func):
        self.init_solution_func = init_solution_func

    def _get_fitness(self, graph):
        makespan = self._get_start_time(graph, self.c_travel, self.c_operate)[-1]
        # fitness = 1/(makespan+0.000001)
        fitness = makespan
        return fitness

    def _get_neighbors(self,graph, affected_add, affected_drop):
        neighbors = []
        neighbors += self._get_n_reroute_neighbors(
            graph, cfg.TABU_NUM_REROUTE_NEIGHBORS, affected_add, affected_drop)
        neighbors += self._get_n_reverse_neighbors(
            graph, cfg.TABU_NUM_REVERSE_NEIGHBORS, affected_add, affected_drop)
        return neighbors

    def _get_n_redistribute_neighbors(self, graph, max_neighbors, affected_redistribute):
        pass
        #TODO

    def redistribute(self, graph, topo_order, cut_after, Ls):
        """Alain's idea of cut and redistribute

        Parameters
        ----------
        graph : _type_
            _description_
        topo_order : _type_
            _description_
        cut_after : _type_
            _description_
        """
        L_bar, L_star = Ls

    def _get_n_redistribute(self,graph, n):
        topo_order = list(nx.topological_sort(graph))
        w,z = self._get_related_times(graph, self.c_travel, self.c_operate)
        # calculate L
        # calculate L_bar
        # calculate L*
        # cut_after = [random(n) for _ in range(n)]
        # for i in n:
        #     neighbor = redistribute(graph,topo_order,cut_after)
        #     neighbors.append(neighbor)
        # re

    def _get_n_reverse_neighbors(self, graph, max_neighbors, affected_add, affected_drop, on_critical = cfg.TABU_REVERSE_ON_CRITICAL):
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
            if self._reversable(edge, ni_descendants):
                G_p = self._reverse(edge, graph, related_times)
                neighbors.append(G_p)
                affected_add[G_p] = [edge]
                affected_drop[G_p] = [(edge[1],edge[0])]
                num_neighbors += 1
                if num_neighbors == max_neighbors:
                    return neighbors
        
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
        u_a = self._get_u(edge, graph, related_times)
        v_b = self._get_v(edge, graph, related_times)
        graph_prime = copy.deepcopy(graph)
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
                edge1 = edges_i[i]
                edge2 = edges_j[j]
                if self._rerouteable(edge1, edge2, descendants):
                    G_p = copy.deepcopy(graph)
                    G_p = self._reroute(G_p, edge1, edge2)
                    neighbors.append(G_p)
                    affected_add[G_p] = [edge1, edge2]
                    affected_drop[G_p] = [(edge1[0],edge2[1]),(edge2[0],edge1[1])]
                    num_neighbors += 1
                    if num_neighbors == max_neighbors:
                        return neighbors
        
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
        # c_travel = copy.deepcopy(c_travel)
        # c_operate = copy.deepcopy(c_operate)
        
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
        return z, waits

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
        #     init_sol = init_instance.solve(
        #     total_robots,
        #     cfg.TABU_INIT_TASK_EACH_ITERATION,
        #     cfg.TABU_INIT_CONSIDER_PROCESS_TIME
        # )
                solve_viet = VGeneralizedSolver(self.processed, self.c_travel, self.c_operate)
                cur_sol = solve_viet.solve(total_robots,'random',cfg.TABU_INIT_CONSIDER_PROCESS_TIME,task_range=[1,4])
                makespan = cur_sol.get_makespan()
        
                if makespan < cur_best_makespan:
                    init_sol  = cur_sol
                    cur_best_makespan = makespan
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

    def _log_config(self):
        if cfg.TABU_LOG_PARAMETERES:
            logging.info(f'TABU SEARCH PARAMETERS')
            logging.info(f'RANDOM_SEED: {cfg.RND_SEED}')
            logging.info(f'NUM_REVERSE_NEIGHBORS: {cfg.TABU_NUM_REVERSE_NEIGHBORS}')
            logging.info(f'NUM_REROUTE_NEIGHBORS: {cfg.TABU_NUM_REROUTE_NEIGHBORS}')
            logging.info(f'MAX_TL_ADD_SIZE: {cfg.TABU_MAX_TL_ADD_SIZE}')
            logging.info(f'MAX_TL_DROP_SIZE: {cfg.TABU_MAX_TL_DROP_SIZE}')
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
                    f'MAX_TL_ADD_SIZE: {cfg.TABU_MAX_TL_ADD_SIZE}',
                    f'MAX_TL_DROP_SIZE: {cfg.TABU_MAX_TL_DROP_SIZE}',
                    f'MAX_ITER: {cfg.TABU_MAX_ITER}',
                    f'FIND_FROM_ELITE_AFTER: {cfg.TABU_FIND_FROM_ELITE_AFTER}',
                    f'RESTART_AFTER: {cfg.TABU_RESTART_AFTER}',
                    f'REROUTE_ON_CRITICAL: {cfg.TABU_REROUTE_ON_CRITICAL}',
                    f'REVERSE_ON_CRITICAL: {cfg.TABU_REVERSE_ON_CRITICAL}',
                    f'REVERSE_CRITICAL_LONGEST_EDGE: {cfg.TABU_REROUTE_LONGEST_EDGE}',
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
        init_sol_makespan = init_sol.get_makespan()
        logging.info(f"Initial makespan: {init_sol_makespan}")
        idx_paths = init_sol.get_idx_paths(change_sink_idx = True)
        G = self._to_graph(idx_paths)

        best_solution = G
        lastest_best_solution = G
        best_solution_fitness = self._get_fitness(best_solution)
        tl_add = []
        tl_drop = []
        i_no_update = 0
        best_fitnesses = [best_solution_fitness]
        cur_fitnesses = [best_solution_fitness]
        for i in range(max_iters):
            affected_add = {} #key : graph, value:edges to add to tl_add
            affected_drop = {} #key : graph, value:edges to add to tl_drop
            neighbors = self._get_neighbors(lastest_best_solution, affected_add, affected_drop)
            best_neighbor = None
            best_neighbor_fitness = np.inf

            for neighbor in neighbors:
                if  not self._is_tabu(affected_add[neighbor], affected_drop[neighbor],tl_add, tl_drop): #TODO find a way to check this quickly
                    neighbor_fitness = self._get_fitness(neighbor)
                    if best_neighbor is None or neighbor_fitness < best_neighbor_fitness:
                        best_neighbor = neighbor
                        best_neighbor_fitness = neighbor_fitness

            if best_neighbor is None:
                logging.info(f'No neighbor found')
                break
            if best_neighbor_fitness < best_solution_fitness:
                best_solution = best_neighbor
                best_solution_fitness = best_neighbor_fitness
                logging.info(f'Best fitness updated: {best_solution_fitness}')
                i_no_update = 0
            else:
                i_no_update += 1


            lastest_best_solution = best_neighbor
            tl_add += affected_add[best_neighbor]
            tl_drop += affected_drop[best_neighbor]
            if len(tl_add) > tabu_list_size:
                # logging.info(f"Removing {len(tl_add) - tabu_list_size} item from tb list")
                [tl_add.pop(i) for i in range(len(tl_add) - tabu_list_size)]
            if len(tl_drop) > tabu_list_size:
                [tl_drop.pop(i) for i in range(len(tl_drop) - tabu_list_size)]
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
        logging.info(f'Best solution found: {best_solution_fitness}')
        return best_solution, best_solution_fitness