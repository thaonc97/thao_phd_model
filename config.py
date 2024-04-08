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
LOG_OUTPUT = False
EXPORT_MODEL = True
USE_INDICATOR = False
RELAXED = False
BIG_M = 10**7

GEN_COOP_RATE = 0.2

DATA_SOURCE = 'wr2'

BENDER_STRAT = 0

#VIET SOLVER CONFIG
BIG_M1 = 10**6

LOG_LEVEL = "INFO" # DEBUG < INFO < WARNING < ERROR < CRITICAL < FATAL
#log_fomartter = "%(message)s"
# Nếu muốn thêm thời gian và loglevel thì dùng format bên dưới
LOG_FORMATTER = "%(asctime)s:%(levelname)s:%(message)s"