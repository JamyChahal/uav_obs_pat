import time

import numpy as np
import scipy
import scipy.ndimage as ndimage
import scipy.ndimage.filters as filters
import matplotlib.pyplot as plt


def main2():
    obs_range = 20
    discret_segment_size = 1000

    my_map = np.ones(discret_segment_size * discret_segment_size)
    my_map.resize((discret_segment_size, discret_segment_size))

    my_map[10, 10] = 10

    t = time.time()
    data_max = filters.maximum_filter(my_map, obs_range)
    maxima = (my_map == data_max)

    location = np.where(maxima)
    print(time.time() - t)
    print(location)


def get_observation_window(x_check, y_check, obs_range, discret_segment_size):
    x_min_check = x_check - obs_range
    x_min_check = x_min_check if x_min_check > 0 else 0
    x_max_check = x_check + obs_range
    x_max_check = x_max_check if x_max_check < discret_segment_size else discret_segment_size
    y_min_check = y_check - obs_range
    y_min_check = y_min_check if y_min_check > 0 else 0
    y_max_check = y_check + obs_range
    y_max_check = y_max_check if y_max_check < discret_segment_size else discret_segment_size
    return x_min_check, x_max_check, y_min_check, y_max_check


def main():
    obs_range = 10
    discret_segment_size = 1000

    my_map = np.ones(discret_segment_size * discret_segment_size)
    my_map.resize((discret_segment_size, discret_segment_size))

    my_map[10, 10] = 10

    t = time.time()
    temp_maxima = np.zeros((discret_segment_size, discret_segment_size), dtype=bool)
    ip = []
    it = ((i, j) for i in range(0, discret_segment_size) for j in range(0, discret_segment_size))
    for pixel in it:
        x_min_check, x_max_check, y_min_check, y_max_check = get_observation_window(pixel[0], pixel[1], obs_range,
                                                                                    discret_segment_size)
        if np.any(temp_maxima[x_min_check:x_max_check,
                  y_min_check:y_max_check]):  # There is already a local maxima around
            continue
        else:
            if np.amax(my_map[x_min_check:x_max_check, y_min_check:y_max_check]) == my_map[pixel[0], pixel[1]]:
                temp_maxima[pixel[0], pixel[1]] = True
                ip.append((pixel[0], pixel[1]))

    print(time.time() - t)
    print(ip)

    # Loop other the map, and find the local maxima


if __name__ == '__main__':
    main()
