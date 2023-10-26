import copy
import random

import cv2
import numpy as np


class Map:

    def __init__(self, map_size_x, map_size_y, obs_range, discretization=10):

        self.segment_size_x = map_size_x * 2  # Size of the map in m
        self.segment_size_y = map_size_y * 2  # Size of the map in m
        self.obs_range = int(obs_range * discretization)  # Observation range (m) to pixel
        self.discretization = discretization  # Number of pixel per m
        self.discret_segment_size_x = int(self.segment_size_x * self.discretization)
        self.discret_segment_size_y = int(self.segment_size_y * self.discretization)

        self.x_max = self.discret_segment_size_x - 1
        self.y_max = self.discret_segment_size_y - 1
        self.x_min = 0
        self.y_min = 0

        self.map = np.ones(self.discret_segment_size_x * self.discret_segment_size_y)
        self.map.resize((self.discret_segment_size_x, self.discret_segment_size_y))

    def get_nbr_cells(self):
        return self.map.size

    def get_high_idleness_only_surrounding(self, x, y):
        cells_coord = []
        cells_value = []
        x = x + self.segment_size_x / 2
        y = y + self.segment_size_y / 2
        x_check, y_check = int(x * self.discretization), int(y * self.discretization)

        # Create interval
        # Verify interval
        x_min_check = x_check - self.obs_range - 1
        x_min_check = max(x_min_check, self.x_min)
        x_max_check = x_check + self.obs_range + 1
        x_max_check = min(x_max_check, self.x_max)
        y_min_check = y_check - self.obs_range - 1
        y_min_check = max(y_min_check, self.y_min)
        y_max_check = y_check + self.obs_range + 1
        y_max_check = min(y_max_check, self.y_max)

        # Upper boundaries
        for xx in range(x_min_check, x_max_check):
            xxx, yyy = xx, y_max_check
            cells_coord.append((xxx, yyy))
            cells_value.append(self.map[xxx, yyy])
            xxx, yyy = xx, y_min_check
            cells_coord.append((xxx, yyy))
            cells_value.append(self.map[xxx, yyy])

        for yy in range(y_min_check, y_max_check):
            xxx, yyy = x_max_check, yy
            cells_coord.append((xxx, yyy))
            cells_value.append(self.map[xxx, yyy])
            xxx, yyy = x_min_check, yy
            cells_coord.append((xxx, yyy))
            cells_value.append(self.map[xxx, yyy])

        cells_value = np.array(cells_value)
        index = np.where(cells_value == np.max(cells_value))
        if len(index) > 0:
            r = random.randint(0, len(index[0]) - 1)
            x_coord, y_coord = cells_coord[index[0][r]]
        else:
            x_coord, y_coord = cells_coord[index[0][0]]

        return x_coord, y_coord


    def observe(self, x, y, update_idleness=False):
        """

        :param x: coord x
        :param y: coord y
        :param update_idleness: True if we just evaluate without updating the idleness
        :return: sum of idleness (negative form)
        """
        f = 0

        # Let reposition x and y [-map_size;map_size] into the map frame [0; map_size²]
        x = x + self.segment_size_x / 2
        y = y + self.segment_size_y / 2

        x_check, y_check = int(x * self.discretization), int(y * self.discretization)
        # Count the number of ones in the interval

        # Verify interval
        x_min_check = x_check - self.obs_range
        x_min_check = x_min_check if x_min_check > 0 else 0
        x_max_check = x_check + self.obs_range
        x_max_check = x_max_check if x_max_check < self.discret_segment_size_x else self.discret_segment_size_x
        y_min_check = y_check - self.obs_range
        y_min_check = y_min_check if y_min_check > 0 else 0
        y_max_check = y_check + self.obs_range
        y_max_check = y_max_check if y_max_check < self.discret_segment_size_y else self.discret_segment_size_y

        f -= np.sum(self.map[x_min_check:x_max_check,
                    y_min_check:y_max_check])
        if update_idleness:
            # Change all values to 0
            self.map[x_min_check:x_max_check, y_min_check:y_max_check] = 0

        return f

    def reset_random(self):
        self.map = np.random.randint(100, size=(self.discret_segment_size_x, self.discret_segment_size_y))

    def new_time(self):
        self.map += 1

    def get_map(self):
        return self.map

    def set_map(self, new_map):
        self.map = new_map

    def update_from_map(self, other_map):
        self.map = np.where(self.map > other_map, other_map, self.map)

    def display_map(self):
        map_copy = copy.copy(self.get_map())
        map_copy[map_copy > 255] = 255

        map_copy = map_copy.astype(np.uint8)
        map_copy = np.rot90(map_copy, k=1, axes=(0, 1))

        img = np.ones([self.discret_segment_size_x, self.discret_segment_size_y, 3])
        img[:, :, 0] = map_copy/255
        img[:, :, 1] = map_copy/255
        img[:, :, 2] = map_copy/255

        img = cv2.resize(img, (900, 900))
        cv2.imshow('image', img)
        k = cv2.waitKey(33)
        if k == 27:  # Esc key to stop
            exit()
        '''
        plt.imshow(np.rot90(map_copy, k=1, axes=(0, 1)), interpolation='nearest')
        plt.show(block=False)
        plt.pause(0.00001)
        '''
