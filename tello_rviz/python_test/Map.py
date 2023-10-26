import copy
import math
import time

import cv2
import numpy as np
from scipy.ndimage import filters


class Map_Idleness:
    def __init__(self, map_size, obs_range, discretization=10):

        self.segment_size = map_size * 2  # Size of the map in m
        self.obs_range = obs_range * discretization  # Observation range (m) to pixel
        self.discretization = discretization  # Number of pixel per m
        self.discret_segment_size = self.segment_size * self.discretization

        self.x_max = self.discret_segment_size
        self.y_max = self.discret_segment_size
        self.x_min = 0
        self.y_min = 0

        self.map = np.ones(self.discret_segment_size * self.discret_segment_size)
        self.map.resize((self.discret_segment_size, self.discret_segment_size))

        self.map_weight = np.ones(self.discret_segment_size * self.discret_segment_size)
        self.map_weight.resize((self.discret_segment_size, self.discret_segment_size))

    def get_segment_size(self):
        return self.segment_size

    def get_discret_segment_size(self):
        return self.discret_segment_size

    def get_nbr_cells(self):
        return self.map.size

    def observe_and_check_cells(self, x, y, update_idleness=False):
        """

        :param x: coord x
        :param y: coord y
        :param update_idleness: False if we just evaluate without updating the idleness
        :return: sum of idleness, max of idleness seen, nbr of cells
        """

        # Check if the agent is within the environment, otherwise no observation at all
        map_size = self.segment_size / 2
        if x < - map_size or x > map_size or y < - map_size or y > map_size:
            return -1, -1, -1

        f = 0

        # Let reposition x and y [-map_size;map_size] into the map frame [0; 2xmap_size]
        x = x + self.segment_size / 2
        y = y + self.segment_size / 2

        x_check, y_check = int(x * self.discretization), int(y * self.discretization)
        # Count the number of ones in the interval

        # Verify interval
        # Verify interval
        x_min_check, x_max_check, y_min_check, y_max_check = self.get_observation_window(x_check, y_check)
        sub_map = self.map[x_min_check:x_max_check, y_min_check:y_max_check]
        nbr_cells = sub_map.size
        if nbr_cells > 0:
            f = np.sum(sub_map)
            m = np.max(sub_map)
        else:
            f = 0
            m = 0

        if update_idleness:
            # Change all values to 0
            self.map[x_min_check:x_max_check, y_min_check:y_max_check] = 0

        return f, m, nbr_cells

    def observe(self, x, y, update_idleness=False):
        """

        :param x: coord x
        :param y: coord y
        :param update_idleness: True if we just evaluate without updating the idleness
        :return: sum of idleness
        """
        f = 0

        # Let reposition x and y [-map_size;map_size] into the map frame [0; map_size²]
        x = x + self.segment_size / 2
        y = y + self.segment_size / 2

        x_check, y_check = int(x * self.discretization), int(y * self.discretization)
        # Count the number of ones in the interval

        # Verify interval
        x_min_check, x_max_check, y_min_check, y_max_check = self.get_observation_window(x_check, y_check)

        f = np.sum(self.map[x_min_check:x_max_check, y_min_check:y_max_check])
        if update_idleness:
            # Change all values to 0
            self.map[x_min_check:x_max_check, y_min_check:y_max_check] = 0

        return f

    def get_observation_window(self, x_check, y_check):
        x_min_check = x_check - self.obs_range
        x_min_check = x_min_check if x_min_check > 0 else 0
        x_max_check = x_check + self.obs_range
        x_max_check = x_max_check if x_max_check < self.discret_segment_size else self.discret_segment_size
        y_min_check = y_check - self.obs_range
        y_min_check = y_min_check if y_min_check > 0 else 0
        y_max_check = y_check + self.obs_range
        y_max_check = y_max_check if y_max_check < self.discret_segment_size else self.discret_segment_size
        return x_min_check, x_max_check, y_min_check, y_max_check

    def reset_random(self, max_value=100):
        self.map = np.random.randint(max_value, size=(self.discret_segment_size, self.discret_segment_size))

    def reset_value(self, value):
        self.map = np.ones((self.discret_segment_size, self.discret_segment_size)) * int(value)

    def new_time(self):
        self.map += 1

    def get_max(self):
        return np.amax(self.map)

    def get_map(self):
        return self.map

    def get_average(self):
        return np.average(self.map)

    def set_map(self, new_map):
        self.map = new_map

    def update_from_map(self, other_map):
        self.map = np.where(self.map > other_map, other_map, self.map)

    def distance(self, x, y, xx, yy):
        return math.sqrt(math.pow(x - xx, 2) + math.pow(y - yy, 2)) / self.discretization

    def compute_map_weighted(self, x, y):
        map_dist = np.ones(self.discret_segment_size * self.discret_segment_size)
        map_dist.resize((self.discret_segment_size, self.discret_segment_size))

        # Let reposition x and y [-map_size;map_size] into the map frame [0; map_size²]
        x = x + self.segment_size / 2
        y = y + self.segment_size / 2

        # Pose of the agent in pixel
        x_check, y_check = int(x * self.discretization), int(y * self.discretization)
        for xx in range(0, self.discret_segment_size):
            for yy in range(0, self.discret_segment_size):
                map_dist[xx][yy] = self.distance(x_check, y_check, xx, yy)

        for xx in range(0, self.discret_segment_size):
            for yy in range(0, self.discret_segment_size):
                if map_dist[xx][yy] > 0:
                    self.map_weight[xx][yy] = self.map[xx][yy] / map_dist[xx][yy]

        return self.map_weight

    def get_map_weighted(self):
        return self.map_weight

    def get_highest_closed_idleness(self):
        return 1

    def display_weight_map(self):
        map_copy = copy.copy(self.map_weight)
        # map_copy[map_copy > 255] = 255

        cv2.normalize(map_copy, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

        map_copy = map_copy.astype(np.uint8)
        map_copy = np.rot90(map_copy, k=1, axes=(0, 1))

        img = np.ones([self.discret_segment_size, self.discret_segment_size, 3])
        img[:, :, 0] = map_copy
        img[:, :, 1] = map_copy
        img[:, :, 2] = map_copy

        res = cv2.resize(img, dsize=(700, 700), interpolation=cv2.INTER_CUBIC)
        cv2.imshow('image', res)
        k = cv2.waitKey(33)
        if k == 27:  # Esc key to stop
            exit()

    def display_idleness_map(self):
        map_copy = copy.copy(self.get_map())
        map_copy[map_copy > 255] = 255

        map_copy = map_copy.astype(np.uint8)
        map_copy = np.rot90(map_copy, k=1, axes=(0, 1))

        img = np.ones([self.discret_segment_size, self.discret_segment_size, 3])
        img[:, :, 0] = map_copy / 255
        img[:, :, 1] = map_copy / 255
        img[:, :, 2] = map_copy / 255

        res = cv2.resize(img, dsize=(700, 700), interpolation=cv2.INTER_CUBIC)
        cv2.imshow('image', res)
        k = cv2.waitKey(33)
        if k == 27:  # Esc key to stop
            exit()

    def is_point_local_max(self, x, y, ip):
        """

        :param x: pixel coordinate of the cell
        :param y: idem
        :param ip: list of the interest point, in pixel unit
        :return: if the point is a local max
        """
        ret = True
        m = self.get_map().copy()
        evaluated_point_value = m[x][y]
        x_min_check, x_max_check, y_min_check, y_max_check = self.get_observation_window(x, y)
        # Check if, in the surrounding, there is other extremum
        pixel_it = ((i, j) for i in range(x_min_check, x_max_check) for j in range(y_min_check, y_max_check))
        for xx, yy in pixel_it:
            if m[xx][yy] > evaluated_point_value or (xx, yy) in ip:
                # There is another point with a maximum, or we have already an interest point in the area
                ret = False
                break
        return ret

    def get_idleness_point_list(self, ip_ij):
        m = self.get_map().copy()
        idleness = []
        for ipp in ip_ij:
            idleness.append(m[ipp[0]][ipp[1]])
        return idleness

    def update_interest_point_2(self, ip_xy):
        """Attempt to improve the speed"""
        ip_ij = self.interest_point_list_into_pixels(ip_xy.copy())
        actual_map = self.get_map().copy()
        data_max = filters.maximum_filter(actual_map, self.obs_range)
        maxima = (actual_map == data_max)

    def update_interest_point(self, ip_xy):
        ip_ij = self.interest_point_list_into_pixels(ip_xy.copy())
        # Scan the map and find the extremum
        # TODO : Compare with the regional map

        for i in range(0, self.discret_segment_size):
            for j in range(0, self.discret_segment_size):
                # Verify interval
                if self.is_point_local_max(i, j, ip_ij):
                    ip_ij.append((i, j))

        return self.interest_point_list_into_x_y(ip_ij), self.get_idleness_point_list(ip_ij)

    def interest_point_list_into_x_y(self, ip_ij: list):
        """
        Used to get the pixel coordinate into x, y coordinate
        :param ip_ij: interest point list (i, j)
        :return: interest point list (x, y)
        """
        ip_xy = []
        for ipp in ip_ij:
            ip_xy.append(self.interest_point_into_x_y(ipp))
        return ip_xy

    def interest_point_into_x_y(self, ipp):
        return (ipp[0] / self.discretization - self.segment_size / 2,
                          ipp[1] / self.discretization - self.segment_size / 2)

    def interest_point_list_into_pixels(self, ip_xy: list):
        """
        Used to get the x, y coordinate into pixel coordinate
        :param ip_xy: interest point list (x, y)
        :return: interest point list (i, j)
        """
        ip_px = []
        for ipp in ip_xy:
            ip_px.append(self.interest_point_into_pixels(ipp))
        return ip_px

    def interest_point_into_pixels(self, ipp):
        return (int((ipp[0] + self.segment_size / 2) * self.discretization),
                int((ipp[1] + self.segment_size / 2) * self.discretization))

    def remove_old_interest_point(self, ip_xy):
        ip_xy = self.interest_point_list_into_pixels(ip_xy)
        for ipp in ip_xy:
            if not self.is_point_local_max(ipp[0], ipp[1], []):
                ip_xy.remove((ipp[0], ipp[1]))
        return self.interest_point_list_into_x_y(ip_xy)