NODE_TYPE = {
    0:'DEPOT',
    1:'PICKUP',
    2:'DESTINATION',
    3:'SINK'
}

PATH_COLOR = {i: f'C{i}' for i in range(10)}

RND_SEED = 5

#SOLVER config
SOLVE_GAP = 0.1
SOLVE_TIME = 3600

BIG_M = 10**6