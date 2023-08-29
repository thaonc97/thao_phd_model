NODE_TYPE = {
    0:'DEPOT',
    1:'PICKUP',
    2:'DESTINATION',
    3:'SINK'
}

PATH_COLOR = {i: f'C{i}' for i in range(10)}

DATA_PATH = 'data/'
RND_SEED = 15

#SOLVER config
SOLVE_GAP = 0.0
SOLVE_TIME = 3600
LOG_OUTPUT = True
EXPORT_MODEL = True
USE_INDICATOR = False
RELAXED = False
BIG_M = 10**6

BENDER_STRAT = 0