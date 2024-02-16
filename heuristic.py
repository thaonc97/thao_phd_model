import copy
import networkx as nx
import numpy as np
import pandas as pd

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
        # print(path)
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
        descendants = {node: nx.descendants(G,node) for node in G.nodes}
        edges = list(G.edges())
        # get 2 different elements from edges:
        for i in range(len(edges)):
            for j in range(i+1,len(edges)):
                edge1 = edges[i]
                edge2 = edges[j]
                if self._two_opt_able(edge1, edge2, descendants) == True:
                    G_p = copy.deepcopy(G)
                    G_p = self._apply_2opt(G_p, edge1, edge2)
                    new_C_max = self._get_start_time(G_p,self.c_travel, self.c_operate)[-1]
                    if new_C_max < cur_C_max:
                        cur_C_max = new_C_max
                        print(f"Applied 2-opt successfully on edges {edge1} and {edge2}, new makespan: {cur_C_max}")
                        G = G_p

        return G
    
    #TODO Error with 10 robots and data_name = 'lim/lrc106-5-uniform.csv'