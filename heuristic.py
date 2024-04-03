import copy
from docplex.mp.model import Model
import heapq
import networkx as nx
import numpy as np
import pandas as pd

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
            # get the n th smallest cost_if_insert
            # n = idx_task_mapper[task_index].n
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
    
    #TODO Error with 10 robots and data_name = 'lim/lrc106-5-uniform.csv'
    #This is because c_travel[0][-1] didn't exsist. I added it manually in the get_start_time function



class VietSolver:
    """Solve PDPCR applying Viet's idea
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
        ns = [task.n for task in self.tasks_remains]
        k_largest = heapq.nlargest(k,ns)
        J_k = sum(k_largest)
        return J_k

    def _solve_viet(self, 
                    total_robots, 
                    tasks_each_iteration, 
                    consider_process_time):
        
        starting_times = {i:0 for i in range(total_robots)}
        J_k = self._cal_Jk(tasks_each_iteration)
        if J_k <= total_robots:
            print(f'J_k = {J_k} <= total_robots = {total_robots}, solving first case' )
            self._lastest_solve_result = self._solve_third_case(total_robots, 
                                                                starting_times, 
                                                                tasks_each_iteration, 
                                                                consider_process_time)
        else: 
            print(f'J_k = {J_k} > total_robots = {total_robots}, solving second case' )
            self._lastest_solve_result = self._solve_second_case(total_robots, 
                                                                 starting_times, 
                                                                 tasks_each_iteration, 
                                                                 consider_process_time)
        #TODO
    
    def _update_paths(self):
        """Update the paths of each robot as well as procesed time
        """
        max_times = {task.index: 0 for task in self.tasks_remains}
        affected_task_idcs = []
        x_results = self._lastest_solve_result['sol'].get_value_dict(self._lastest_solve_result['x'])
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
                    cur_time = self.c_travel[cur_path_last_idx][task_idx] 
                    max_times[task_idx] = max(max_times[task_idx], cur_time)
                    self.paths[robot_idx].insert(self.idx_task_mapper[task_idx])
                except Exception as e:
                    print(e)
                    pass
        
        affected_task_idcs = set(affected_task_idcs)
        for path_idx,path in self.paths.items():
            last_task_idcs = path[-1].index
            if last_task_idcs in affected_task_idcs:
                time_to_add =max_times[last_task_idcs] +self.c_operate[last_task_idcs]
                path.add_total_time(time_to_add)
        
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
            logging.debug(f'Start solving')
            self._solve_viet(total_robots, tasks_each_iteration, consider_process_time)
            logging.debug(f'Solved!\n updating paths...')
            self._update_paths()
            logging.debug(f'Done\n Removing processed tasks...')
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
            a[i] = {j:cfg.BIG_M1 for j in range(1,len(self.tasks_remains) + 1)}
        return a

    def _solve_second_case(self,
                           total_robots,
                           starting_times :dict,
                           tasks_each_iteration : int,
                           consider_process_time : bool):
        
        # total_robots_required = np.sum([task.n for task in self.tasks_remains]) # This is probably for assign all 
                                                                                # robots at each iteration case
        J_k = self._cal_Jk(tasks_each_iteration) 
        A_idcs = [i for i in range(1, len(self.tasks_remains) + 1)]
        print(A_idcs)
        B_idcs = [j for j in range(total_robots)]
        Bf_idcs = [j for j in range(total_robots, J_k)] #Artificial robots
        Bp_idcs = B_idcs + Bf_idcs
        BpA_idcs = [(i,j) for i in Bp_idcs for j in A_idcs]

        k = tasks_each_iteration
        a = self._construct_a_artificial(total_robots, J_k)
        t = starting_times
        c = self.c_operate
        n ={task.index: task.n for task in self.tasks_remains}

        m = Model('Viet_case_2')
        C_max = m.continuous_var(name = 'C_max')
        x = m.binary_var_dict(BpA_idcs, name = 'x')
        # f = m.continuous_var_dict(BpA_idcs, name = 'f')
        v= m.binary_var_dict(A_idcs, name = 'v')
        q = m.binary_var_dict(A_idcs, name = 'q')
        d = m.continuous_var(name = 'd', lb = 0)
        p = m.continuous_var_dict(BpA_idcs,name = 'p') #fake variable

        m.minimize(C_max)
        if consider_process_time:
            m.add_constraints(C_max + (a[i][j] + t[i] + c[j])*v[j] >= (a[i][j]+ t[i] + c[j])*x[i,j] for i in B_idcs for j in A_idcs)
        else:
            m.add_constraints(C_max + (a[i][j] + t[i])*v[j] >= (a[i][j]+ t[i])*x[i,j] for i in B_idcs for j in A_idcs)

        m.add_constraints(C_max >= cfg.BIG_M1*v[j] - cfg.BIG_M1*p[i,j] for j in A_idcs for i in Bp_idcs)
        m.add_constraints(m.sum(x[i,j] for i in Bp_idcs) == n[j] for j in A_idcs)
        m.add_constraints(m.sum(x[i,j] for j in A_idcs) <= 1 for i in Bp_idcs)
        m.add_constraints(x[i,j] <= 1 for i in Bp_idcs for j in A_idcs) #Capacity constraint
        m.add_constraint(m.sum((m.sum(x[i,j] for j in A_idcs) for i in Bp_idcs)) == d) #Required flow
        m.add_constraint(m.sum(q[j] for j in A_idcs) == k)
        m.add_constraints(x[i,j] <= q[j] for i in Bp_idcs for j in A_idcs) # Relation x and q
        # m.add_constraints(x[i,j] <= q[j] for i in Bp_idcs for j in A_idcs) # Relation x and q
        m.add_constraints(x[i,j] <= v[j] for i in Bf_idcs for j in A_idcs) # Relation x and v
        m.add_constraint(m.sum(q[j]*n[j] for j in A_idcs) == d) #Acquire flow

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


    def _solve_third_case(self, 
                          total_robots, 
                          starting_times :dict, 
                          tasks_each_iteration : int, 
                          consider_process_time : bool):
        A_idcs = [i for i in range(1,len(self.tasks_remains) + 1)]
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
    

class VietSolverOrig(VietSolver):
    """Class for solve Viet's problem wiht trying to assign every robots
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
            self._lastest_solve_result = self._solve_second_case(total_robots, 
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
            m.add_constraints(C_max >= (a[i][j] + t[i] + c[j])*x[i,j] + cfg.BIG_M1*v[j] for i in B_idcs for j in A_idcs)
        else:
            # m.add_constraints(C_max + (a[i][j] + t[i])*v[j] >= (a[i][j]+ t[i])*x[i,j] for i in B_idcs for j in A_idcs)
            m.add_constraints(C_max  >= (a[i][j]+ t[i])*x[i,j] + cfg.BIG_M1*v[j] for i in B_idcs for j in A_idcs)

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