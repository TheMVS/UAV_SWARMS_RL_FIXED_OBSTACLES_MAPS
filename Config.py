# -*- coding: utf-8 -*-

SEED = None  # None if we want a random seed or a number if we want to specify a seed

LOAD_MAP_FILE = True  # For loading map from a file

CONTINUE_EXPERIMENT = False  # True if we want to resume experimentation (in case of timeout, error...)
MAX_EXPERIMENT_TIME = 100  # Max hours for saving status of that experiment (Data Centers with maximun execution time)

# FILES PATH
BASE_ROUTE = './'
DATA_ROUTE = 'data.json'
MAP_ROUTE = 'maps/map_7_7.txt'
EPSILONS_ROUTE = 'epsilons.csv'
SIMULATIONS_TIMES_ROUTE = 'simulations_times.csv'
SIMULATIONS_REWARDS = 'simulations_rewards.csv'
SIMULATIONS_COVERAGES = 'simulations_coverages.csv'
SIMULATION_COVERAGES = 'simulation_coverages.csv'

# Episode configuration
EPISODES = 30
PRINT_EPISODES = False  # Print for checkpoint on console
SIMULATIONS_CHECKPOINT = 100  # Number of episodes that must happen before printing checkpoint

# Environment configuration
ENVIRONMENT_ROWS = 7
ENVIRONMENT_COLUMNS = 7
SQUARE = True  # Make environment polygon squared
ADJUST_ENVIRONMENT = False  # Adjust environment grid to drones' image size
START_CORNER_0_0 = True
USE_RANDOM_OBSTACLES = True
NUMBER_OBSTACLES = ENVIRONMENT_ROWS # Fixed number of obstacles

# Agent Configuration
ACTIONS_DICT = {
    0: 'left',
    1: 'up',
    2: 'right',
    3: 'down',
}
NEW_CELL_REWARD = 358.736076826821
VISITED_CELL_REWARD = -31.1376955791041
NO_CELL_REWARD = -225.171111437135

# Learning process configuration
GLOBAL_MODEL = False
EPSILON = 0.468937067929711 # 0.498082374999
MIN_EPSILON = 0.05
GAMMA = 0.902865796260127
MEMORY_SIZE = 60
BATCH_SIZE = 63
EPOCHS = 2
VERBOSE = 0
EPSILON_DECAY = 0.929049010143763

# Model
CONV1_FILTER_NUMBER = 256
CONV1_SIZE = (3,3)
CONV1_STRIDE = (1,1)
CONV1_ACTIVATION = 'relu'
CONV2_FILTER_NUMBER = 256
CONV2_SIZE = (3,3)
CONV2_STRIDE = (1,1)
CONV2_ACTIVATION = 'relu'
DENSE1_SIZE = 167
DENSE1_ACTIVATION = 'linear'
DENSE2_SIZE = len(ACTIONS_DICT)
DENSE2_ACTIVATION = 'softmax'
OPTIMIZER = 'RMSprop'

# Early experiment stopping
MAXIMUM_WAIT_HOURS = 0.5
COMPLETENESS_COVERAGE = 1.0  # float between 0 and 1
MAXIMUM_UNCHANGED_ENVIRONMENT_EPISODES = 9999999999

# Random Search
USE_RANDOM_SEARCH = False
NUMBER_OF_RANDOM_EXPERIMENTS = 5000

# Unit tests
UNIT_TESTS = False
