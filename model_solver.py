from docplex.mp.model import Model
import numpy as np
import pandas as pd

import config as cfg
import data_processor
from helpers import create_cost_dict


def solve_vrp_sumofcosts(data):
    v_prime,e_prime = data_processor.process_vrp_sumofcosts(data)

def cplex_solve_vrp_sumofcosts(total_robots,
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
    model.parameters.mip.tolerances.mipgap = cfg.SOLVE_GAP
    model.time_limit = cfg.SOLVE_TIME
    model.minimize(model.sum(c[i][j]*x[i,j] for i in i_idx for j in e_prime[i]) + model.sum(y[i.index] for i in v_prime))
    sol = model.solve(log_output = True)
    
    solved_gap = model.solve_details.mip_relative_gap
    solve_info = {
        'sol':sol,
        'x':x,
        'y':y,
        'gap':solved_gap
    }
    
    return solve_info