import time

import numpy as np

from .Map import Map_Idleness


def main():
    map_size = 20
    obs_range = 10
    discretization = 1

    M = Map_Idleness(map_size=map_size, obs_range=obs_range, discretization=discretization)
    M.reset_random(100)




if __name__ == '__main__':
    main()
