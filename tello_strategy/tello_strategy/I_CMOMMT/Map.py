import copy

import cv2
import numpy as np


class Map:

    def __init__(self, map_size_x, map_size_y, obs_range, discretization=10):

        self.segment_size_x = map_size_x * 2  # Size of the map in m
        self.segment_size_y = map_size_y * 2  # Size of the map in m
        self.obs_range = int(obs_range * discretization)  # Observation range (m) to pixel
        self.discretization = discretization  # Number of pixel per m
        self.discret_segment_size_x = int(self.segment_size_x * self.discretization - 2)
        self.discret_segment_size_y = int(self.segment_size_y * self.discretization - 2)

        self.x_max = self.discret_segment_size_x
        self.y_max = self.discret_segment_size_y
        self.x_min = 0
        self.y_min = 0

        self.map = np.ones(self.discret_segment_size_x * self.discret_segment_size_y)
        self.map.resize((self.discret_segment_size_x, self.discret_segment_size_y))

    def get_nbr_cells(self):
        return self.map.size

    def observe_and_check_cells(self, x, y, update_idleness=False):
        """

        :param x: coord x
        :param y: coord y
        :param update_idleness: True if we just evaluate without updating the idleness
        :return: sum of idleness (negative form)
        """

        f = 0

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

        nbr_cells = len(self.map[x_min_check:x_max_check, y_min_check:y_max_check])

        f -= np.sum(self.map[x_min_check:x_max_check, y_min_check:y_max_check])
        if update_idleness:
            # Change all values to 0
            self.map[x_min_check:x_max_check, y_min_check:y_max_check] = 0

        return f, nbr_cells

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
