NODE_TYPE = {
    0:'DEPOT',
    1:'PICKUP',
    2:'DESTINATION',
    3:'SINK'
}

PATH_COLOR = {i: f'C{i}' for i in range(10)}

DATA_PATH = 'data/'
RND_SEED = 16

#SOLVER config
SOLVE_GAP = 0.0
SOLVE_TIME = 3600*5
LOG_OUTPUT = True
EXPORT_MODEL = False
USE_INDICATOR = False
RELAXED = False
BIG_M = 10**5

GEN_COOP_RATE = 0.2

DATA_SOURCE = 'wr2'

BENDER_STRAT = -1

#VIET SOLVER CONFIG
BIG_M1 = 10**4

#Config for Tabu seach init
TABU_INIT_TASK_EACH_ITERATION = 1
TABU_INIT_CONSIDER_PROCESS_TIME = False
TABU_MAX_ITER = 3000
TABU_NUM_REROUTE_NEIGHBORS = 20
TABU_NUM_REVERSE_NEIGHBORS = 7
TABU_NUM_REDISTRIBUTE_NEIGHBORS = 6
TABU_MAX_TL_ADD_SIZE = 20
TABU_MAX_TL_DROP_SIZE = 20
TABU_TL_REDISTRIBUTE_SIZE = 10
TABU_LOG_EVERY_ITERATIONS = 50
TABU_LOG_PARAMETERES = True
TABU_FIND_FROM_ELITE_AFTER = 300 #iterations
TABU_RESTART_AFTER = 600 #iterations
TABU_REVERSE_ON_CRITICAL = False
TABU_REROUTE_ON_CRITICAL = False
TABU_REROUTE_LONGEST_EDGE = False
VIET_RANDOM_ITE =30
TABU_REDISTRIBUTE_PROB = 0.5 # The probability to keep best candidate redistribute instead of change to reroute or reverse
TABU_SAVE_FITNESSES = True


GREEDY_DEBUG = True
DEDICATED_INSERT = True
DEBUG_IHAT_NOT_RANDOM = True
LOG_LEVEL = "INFO" # DEBUG < INFO < WARNING < ERROR < CRITICAL < FATAL
#log_fomartter = "%(message)s"
# Nếu muốn thêm thời gian và loglevel thì dùng format bên dưới
LOG_FORMATTER = "%(asctime)s:%(levelname)s:%(message)s"

