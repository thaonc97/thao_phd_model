from docplex.mp.model import Model
import numpy as np
import pandas as pd
import time

import config as cfg
import data_acquirer
import data_processor
import helpers


def solve_vrp_sumofcosts(data,total_robots):
    depot, tasks,v_prime, e_prime = data.unpack()
    solve_result = _cplex_solve_vrp_sumofcosts(total_robots,tasks,v_prime, e_prime)
    return solve_result

def solve_flow_makespan(data,total_robots):
    depot, tasks,nodes, edges = data.unpack()
    solve_result = _cplex_solve_flow_makespan(total_robots, tasks, nodes, edges)
    return solve_result

def solve_vrp_makespan(data, total_robots):
    depot, tasks,v_prime, e_prime = data.unpack()
    solve_result = _cplex_solve_vrp_makespan(total_robots,tasks,v_prime, e_prime)
    return solve_result

def solve_vrp_ver2(data,total_robots):
    depot = data.depot
    expanded_tasks = data.tasks
    solve_result = _cplex_solve_vrp_ver2(total_robots,depot, expanded_tasks)
    return solve_result

def solve_vrp_ver2_makespan(data,total_robots):
    depot = data.depot
    expanded_tasks = data.tasks
    solve_result = _cplex_solve_vrp_ver2_makespan(total_robots,depot, expanded_tasks)
    return solve_result

def _cplex_solve_vrp_sumofcosts(total_robots,
                tasks,
                v_prime, 
                e_prime,  
                log_output =True, 
                handle_indicator = 1):
    
    R = total_robots
    t_idx = [task.index for task in tasks]
    i_idx = [node.index for node in v_prime]
    ij_idx = [(i.index,j) for i in v_prime for j in i.connection]
    Pt = {task.index:[node.index for node in v_prime if (node.task_idx ==task.index and node.node_type ==1)] for task in tasks}
    Dt = {task.index:[node.index for node in v_prime if (node.task_idx ==task.index and node.node_type ==2)] for task in tasks}

    node_idx_mapper = {node.index: node for node in v_prime}
    c = helpers.create_cost_dict(v_prime,node_idx_mapper)

    model = Model("ThaoModel")
    x= model.binary_var_dict(ij_idx, name = 'x')
    y = model.continuous_var_dict(i_idx,lb =0, name = 'y')
    z = model.continuous_var_dict(i_idx,lb = 0, name = 'z')
    
    model.add_constraint(model.sum(x[0,j] for j in e_prime[0]) == R) #2
    model.add_constraint(model.sum(x[i,0] for i in e_prime if 0 in e_prime[i]) == R) #3
    model.add_constraints(model.sum(x[i,j] for i in e_prime if j in e_prime[i])==1 for j in i_idx[1:])
    model.add_constraints(model.sum(x[i,j] for j in e_prime[i])==1 for i in i_idx[1:])
    if handle_indicator ==1:
        indicator_SEC = [model.indicator_constraint(x[i,j],z[j] ==z[i]  +c[i][j] +y[j]) for i  in i_idx for j in e_prime[i] if j!=0]
        model.add_indicator_constraints(indicator_SEC)
        pass
    elif handle_indicator ==2:
        model.add_lazy_constraints(z[j] + cfg.BIG_M*(1-x[i,j]) >= z[i] +c[i][j] + y[j] for i  in i_idx for j in e_prime[i] if j!=0)
        model.add_lazy_constraints(z[j] - cfg.BIG_M*(1-x[i,j]) <= z[i] +c[i][j] + y[j] for i  in i_idx for j in e_prime[i] if j!=0)
    model.add_constraints(z[Pt[t][0]] == z[j] for t in t_idx for j in Pt[t])
    model.add_constraints(z[Dt[t][0]] == z[j] for t in t_idx for j in Dt[t])
    model.minimize(model.sum(c[i][j]*x[i,j] for i in i_idx for j in e_prime[i]) + model.sum(y[i.index] for i in v_prime))

    model.parameters.mip.tolerances.mipgap = cfg.SOLVE_GAP
    model.time_limit = cfg.SOLVE_TIME
    sol = model.solve(log_output = cfg.LOG_OUTPUT)
    
    solved_gap = model.solve_details.mip_relative_gap
    solve_info = {
        'sol':sol,
        'x':x,
        'y':y,
        'gap':solved_gap
    }
    
    return solve_info

def _cplex_solve_flow_makespan(total_robots,
                tasks,
                nodes, 
                edges,
                ):
    
    j_idx = [task.index for task in tasks] #jobs
    u_idx = [node.index for node in nodes] #nodes
    sink_idx = len(nodes) -1 
    uv_idx = [(u.index,v) for u in nodes for v in u.connection] 
    M = total_robots
    
    node_idx_mapper = {node.index: node for node in nodes}
    c = helpers.create_cost_dict(nodes,node_idx_mapper)
    edge_travels = data_processor.construct_edge_travels(edges, node_idx_mapper)
    edge_jobs = data_processor.construct_edge_jobs(edges, node_idx_mapper)
    capac = data_processor.cal_capacity(edge_jobs,edge_travels, node_idx_mapper,tasks)

    model = Model("PDPCR Flow Based Makespan")
    f= model.continuous_var_dict(uv_idx, name = 'f')
    z = model.continuous_var_dict(u_idx,lb = 0, name = 'z')
    if cfg.RELAXED:
        x = model.continuous_var_dict(uv_idx,ub = 1, name = 'x')
    else:
        x = model.binary_var_dict(uv_idx, name = 'x')

    model.add_constraint(model.sum(f[0,v] for v in edges[0] if v != sink_idx) == total_robots) #14
    model.add_constraint(model.sum(f[u,sink_idx] for u in edges if sink_idx in edges[u]) == total_robots) #15
    model.add_constraints(model.sum(f[u,v] for u in u_idx if v in edges[u]) - model.sum(f[v,vp] for vp in edges[v]) == 0 for v in u_idx[1:-1]) #16
    model.add_constraints(f[u,v] == capac[(u,v)] for (u,v) in edge_jobs) #18
    model.add_constraint(z[0] == 0) #19
    model.add_constraints(z[v] == z[u] + c[u][v] for (u,v) in edge_jobs) #20
    model.add_constraints(x[u,v] == 1 for (u,v) in edge_jobs)
    model.add_constraints(M*x[u,v] >= f[u,v] for (u,v) in edge_travels)
    model.add_constraints(f[u,v] >= x[u,v] for (u,v) in edge_travels)
    if cfg.USE_INDICATOR:
        indicator_SEC = [model.indicator_constraint(x[u,v],z[v] >=z[u]  +c[u][v]) for (u,v) in edge_travels] #21
        model.add_indicator_constraints(indicator_SEC) 
    else:
        big_m = helpers.cal_big_m(nodes)
        big_m = cfg.BIG_M
        model.add_constraints(z[v] + big_m*(1-x[u,v]) >= z[u] +c[u][v] for (u,v) in edge_travels)
    model.minimize(z[sink_idx])
    
    if cfg.BENDER_STRAT:
        model.parameters.benders.strategy = cfg.BENDER_STRAT

    model.parameters.mip.tolerances.mipgap = cfg.SOLVE_GAP
    model.time_limit = cfg.SOLVE_TIME
    if cfg.EXPORT_MODEL == True:
        print(model.export_as_lp())
    sol = model.solve(log_output = cfg.LOG_OUTPUT)
    
    solved_gap = model.solve_details.mip_relative_gap
    solve_info = {
        'sol':sol,
        'x':x,
        'f':f,
        # 'y':y,
        'gap':solved_gap
    }
    
    return solve_info

def _cplex_solve_vrp_makespan(total_robots,
                tasks,
                v_prime, 
                e_prime,  
                log_output =True, 
                handle_indicator = 1):
    
    R = total_robots
    t_idx = [task.index for task in tasks]
    i_idx = [node.index for node in v_prime]
    ij_idx = [(i.index,j) for i in v_prime for j in i.connection]
    Pt = {task.index:[node.index for node in v_prime if (node.task_idx ==task.index and node.node_type ==1)] for task in tasks}
    Dt = {task.index:[node.index for node in v_prime if (node.task_idx ==task.index and node.node_type ==2)] for task in tasks}
    
    sink_idx = len(v_prime) -1
    node_idx_mapper = {node.index: node for node in v_prime}
    c = helpers.create_cost_dict(v_prime,node_idx_mapper)

    model = Model("ThaoModel")
    x= model.binary_var_dict(ij_idx, name = 'x')
    y = model.continuous_var_dict(i_idx,lb =0, name = 'y')
    z = model.continuous_var_dict(i_idx,lb = 0, name = 'z')
    
    model.add_constraint(model.sum(x[0,j] for j in e_prime[0]) == R) #2
    model.add_constraint(model.sum(x[i,0] for i in e_prime if 0 in e_prime[i]) == R) #3
    model.add_constraints(model.sum(x[i,j] for i in e_prime if j in e_prime[i])==1 for j in i_idx[1:])
    model.add_constraints(model.sum(x[i,j] for j in e_prime[i])==1 for i in i_idx[1:])
    if handle_indicator ==1:
        indicator_SEC = [model.indicator_constraint(x[i,j],z[j] ==z[i]  +c[i][j] +y[j]) for i  in i_idx for j in e_prime[i] if j!=0]
        model.add_indicator_constraints(indicator_SEC)
        pass
    elif handle_indicator ==2:
        model.add_lazy_constraints(z[j] + cfg.BIG_M*(1-x[i,j]) >= z[i] +c[i][j] + y[j] for i  in i_idx for j in e_prime[i] if j!=0)
        model.add_lazy_constraints(z[j] - cfg.BIG_M*(1-x[i,j]) <= z[i] +c[i][j] + y[j] for i  in i_idx for j in e_prime[i] if j!=0)
    model.add_constraints(z[Pt[t][0]] == z[j] for t in t_idx for j in Pt[t])
    model.add_constraints(z[Dt[t][0]] == z[j] for t in t_idx for j in Dt[t])
    model.minimize(z[sink_idx])

    model.parameters.mip.tolerances.mipgap = cfg.SOLVE_GAP
    model.time_limit = cfg.SOLVE_TIME
    sol = model.solve(log_output = cfg.LOG_OUTPUT)
    
    solved_gap = model.solve_details.mip_relative_gap
    solve_info = {
        'sol':sol,
        'x':x,
        'y':y,
        'gap':solved_gap
    }
    
    return solve_info

def _cplex_solve_vrp_ver2(total_robots,depot, expanded_tasks):
    """
    In this model, the variable associated with pickup-delivery edge is emitted.
    """
    I = total_robots
    JP =  [depot] + expanded_tasks
    i0_idx = [i.index for i in JP]
    i_idx = [i.index for i in JP if i.index != 0]
    ij_idx = [(i.index, j.index) for i in JP for j in JP if i.index != j.index]

    model = Model("VRP ver 2 sum of costs")
    if cfg.RELAXED:
        x = model.continuous_var_dict(ij_idx,ub = 1, name = 'x')
    else:
        x = model.binary_var_dict(ij_idx, name = 'x')
    y = model.continuous_var_dict(i0_idx,lb =0, name = 'y')
    z = model.continuous_var_dict(i0_idx,lb = 0, name = 'z')

    c_travel = helpers.create_task_dist_dict(depot, expanded_tasks)
    c_operate = helpers.create_operate_dict(depot, expanded_tasks)
    orig_task = helpers.create_expanded_original_task_mapper(expanded_tasks)

    model.add_constraint(model.sum(x[0,i] for i in i_idx) == model.sum(x[j,0] for j in i_idx))
    model.add_constraint(model.sum(x[i,0] for i in i_idx) == I)
    model.add_constraints(model.sum(x[i,j] for j in i0_idx if j!= i) == 1 for i in i_idx)
    model.add_constraints(model.sum(x[i,j] for i in i0_idx if i!= j) == 1 for j in i_idx)
    if cfg.USE_INDICATOR:
        SEC = [model.indicator_constraint(x[i,j],z[j] == z[i] + c_travel[i][j] +c_operate[i] +y[j]) for i in i0_idx for j in i_idx if i!=j]
        model.add_indicator_constraints(SEC)
    else:
        big_m = 10**6
        model.add_constraints(z[j] + big_m*(1-x[i,j]) >= z[i] + c_operate[i]  +c_travel[i][j] +y[j] for i in i0_idx for j in i_idx if i!=j)
        model.add_constraints(z[j] - big_m*(1-x[i,j]) <= z[i] + c_operate[i]  +c_travel[i][j] +y[j] for i in i0_idx for j in i_idx if i!=j)
    model.add_constraints(z[i] == z[j] for i in i_idx for j in i_idx  if orig_task[i] == orig_task[j])
    model.minimize(model.sum(c_travel[i][j]*x[i,j] for i in i0_idx for j in i0_idx if i!=j) + model.sum(y[i] for i in i_idx))

    model.parameters.mip.tolerances.mipgap = cfg.SOLVE_GAP
    model.time_limit = cfg.SOLVE_TIME
    if cfg.EXPORT_MODEL == True:
        print(model.export_as_lp())
        
    sol = model.solve(log_output = cfg.LOG_OUTPUT)
    solved_gap = model.solve_details.mip_relative_gap
    solve_info = {
        'sol':sol,
        'x':x,
        'y':y,
        'z':z,
        'gap':solved_gap
    }
    
    return solve_info

def _cplex_solve_vrp_ver2_makespan(total_robots,depot, expanded_tasks):
    """
    In this model, the variable associated with pickup-delivery edge is emitted.
    """
    I = total_robots
    JP =  [depot] + expanded_tasks
    sink_idx = max([task.index for task in expanded_tasks])
    i0_idx = [i.index for i in JP]
    i_idx = [i.index for i in JP if i.index != 0 if i.index!= sink_idx]
    ij_idx = [(i.index, j.index) for i in JP for j in JP if i.index != j.index]
    
    model = Model("VRP ver 2 makespan")
    x= model.binary_var_dict(ij_idx, name = 'x')
    z = model.continuous_var_dict(i0_idx,lb = 0, name = 'z')
    c_max = model.continuous_var(0, name = "c_max")
    c_travel = helpers.create_task_dist_dict(depot, expanded_tasks)
    c_operate = helpers.create_operate_dict(depot, expanded_tasks)
    orig_task = helpers.create_expanded_original_task_mapper(expanded_tasks)

    model.add_constraint(model.sum(x[0,i] for i in i_idx) == model.sum(x[j,sink_idx] for j in i_idx))
    model.add_constraint(model.sum(x[0,i] for i in i_idx) == I)
    model.add_constraints(model.sum(x[i,j] for j in i0_idx if j!= i if j!=0) == 1 for i in i_idx)
    model.add_constraints(model.sum(x[i,j] for i in i0_idx if i!= j if i != sink_idx) == 1 for j in i_idx)
    SEC = [model.indicator_constraint(x[i,j],z[j] >= z[i] + c_travel[i][j] +c_operate[i]) for i in i0_idx if i!=sink_idx for j in i0_idx if i!=j if j!= 0]
    model.add_indicator_constraints(SEC)
    model.add_constraint(z[0] == 0)
    model.add_constraints(z[i] == z[j] for i in i_idx for j in i_idx  if orig_task[i] == orig_task[j])
    model.add_constraints(z[i] <= c_max for i in i0_idx)
    model.minimize(c_max)

    model.parameters.mip.tolerances.mipgap = cfg.SOLVE_GAP
    model.time_limit = cfg.SOLVE_TIME
    if cfg.EXPORT_MODEL == True:
        print(model.export_as_lp())
    print('Start solving vrp-based model for makespan obj.function using CPLEX')
    sol = model.solve(log_output = cfg.LOG_OUTPUT)
    solved_gap = model.solve_details.mip_relative_gap
    solve_info = {
        'sol':sol,
        'x':x,
        'z':z,
        'gap':solved_gap
    }
    
    return solve_info

SOLVE_METHOD ={
    1: solve_vrp_sumofcosts,
    2: solve_flow_makespan,
    3: solve_vrp_makespan,
    4: solve_vrp_ver2,
    5: solve_vrp_ver2_makespan
}

def solve_pdpcr(processed_data, total_robots, method :int):
    """
    SOLVE_METHOD ={
    1: solve_vrp_sumofcosts,
    2: solve_flow_makespan,
    3: solve_vrp_makespan,
    4: solve_vrp_ver2
    }
    """
    solve_method = SOLVE_METHOD[method]
    solve_result = solve_method(processed_data, total_robots)
    return solve_result

class SolvedInstance():
    
    def __init__(self, seed, data, total_robots, sol_result, solve_time,instance_num):
        self.seed = seed
        self.data = data
        self.total_robots = total_robots
        self.solve_time = solve_time
        self.gap = self._get_gap(sol_result)
        self.solve_result = sol_result
        self.instance_num = instance_num
        
    def _get_gap(self, solve_result):
        return solve_result['gap']


def mass_solve(total_tasks,total_robots,method,seed =1, total_instances = 5, log_output = True):
    solved_instances = []
    for instance_num in range(total_instances):
        depot_as_task = False
        if method == 4 or method ==5:
            depot_as_task = True
        raw = data_acquirer.gen_data(total_robots, total_tasks, False,depot_as_task)
        processed_data = data_processor.process(raw, method) 
        try:
            tik = time.time()
            solved_result = solve_pdpcr(processed_data, total_robots, method)
            tok = time.time()
            solved_time = tok - tik
            if solved_result['sol'] is None:
                print("No solution!")
        except:
            print(f"There is problem solving {total_tasks} tasks, {total_robots} robots,seed{seed}")
            continue
        try:
            solved_instance = SolvedInstance(seed, processed_data, total_robots, solved_result,solved_time, instance_num)
            solved_instances.append(solved_instance)
        except Exception as e:
            raise e
        
    return solved_instances