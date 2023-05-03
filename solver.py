from docplex.mp.model import Model
import numpy as np
import pandas as pd

import config as cfg
import data_processor
from helpers import create_cost_dict


def solve_vrp_sumofcosts(data,total_robots):
    depot, tasks = data
    v_prime,e_prime = data_processor.process_vrp_sumofcosts(data)
    solve_result = _cplex_solve_vrp_sumofcosts(total_robots,tasks,v_prime, e_prime)
    return solve_result

def solve_flow_makespan(data,total_robots):
    depot, tasks = data
    nodes, edges = data_processor.process_flow(data)
    solve_result = _cplex_solve_flow_makespan(total_robots, tasks, nodes, edges)
    return solve_result

def solve_vrp_makespan(data, total_robots):
    depot, tasks = data
    v_prime,e_prime = data_processor.process_vrp_sumofcosts(data)
    solve_result = _cplex_solve_vrp_makespan(total_robots,tasks,v_prime, e_prime)
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
    c = create_cost_dict(v_prime,node_idx_mapper)

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
    c = create_cost_dict(nodes,node_idx_mapper)
    edge_travels = data_processor.construct_edge_travels(edges, node_idx_mapper)
    edge_jobs = data_processor.construct_edge_jobs(edges, node_idx_mapper)
    capac = data_processor.cal_capacity(edge_jobs,edge_travels, node_idx_mapper,tasks)

    model = Model("PDPCR Flow Based Makespan")
    f= model.integer_var_dict(uv_idx, name = 'f')
    # y = model.continuous_var_dict(uv_idx,lb =0, name = 'y')
    z = model.continuous_var_dict(u_idx,lb = 0, name = 'z')
    x = model.binary_var_dict(uv_idx, name = 'x')

    model.add_constraint(model.sum(f[0,v] for v in edges[0]) == total_robots) #14
    model.add_constraint(model.sum(f[u,sink_idx] for u in edges if sink_idx in edges[u]) == total_robots) #15
    model.add_constraints(model.sum(f[u,v] for u in u_idx if v in edges[u]) - model.sum(f[v,vp] for vp in edges[v]) == 0 for v in u_idx[1:-1]) #16
    model.add_constraints(f[u,v] == capac[(u,v)] for (u,v) in edge_jobs) #18
    model.add_constraint(z[0] == 0) #19
    model.add_constraints(z[v] == z[u] + c[u][v] for (u,v) in edge_jobs) #20
    model.add_constraints(M*x[u,v] >= f[u,v] for (u,v) in edge_travels)
    model.add_constraints(f[u,v] >= x[u,v] for (u,v) in edge_travels)
    indicator_SEC = [model.indicator_constraint(x[u,v],z[v] >=z[u]  +c[u][v]) for (u,v) in edge_travels] #21
    model.add_indicator_constraints(indicator_SEC) 

    model.minimize(z[sink_idx])
    
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
    c = create_cost_dict(v_prime,node_idx_mapper)

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

SOLVE_METHOD ={
    1: solve_vrp_sumofcosts,
    2: solve_flow_makespan,
    3: solve_vrp_makespan,
}

def solve_pdpcr(data, total_robots, method):
    """
    SOLVE_METHOD ={
    1: solve_vrp_sumofcosts,
    2: solve_flow_makespan,
    3: solve_vrp_makespan,
    }
    """
    solve_method = SOLVE_METHOD[method]
    solve_result = solve_method(data, total_robots)
    return solve_result