import copy

import cv2
import numpy as np


class Map_Env:

    def __init__(self, map_size, obs_range, com_range, discretization=1):

        self.map_size = map_size
        self.obs_range = obs_range
        self.com_range = com_range
        self.segment_size = map_size * 2  # Size of the map in m
        self.discretization = discretization  # Number of pixel per m
        self.discret_segment_size = int(self.segment_size * self.discretization)

        self.x_max = self.discret_segment_size
        self.y_max = self.discret_segment_size
        self.x_min = 0
        self.y_min = 0

        self.map_idleness = np.zeros(self.discret_segment_size * self.discret_segment_size).astype(np.float32)
        self.map_idleness.resize((self.discret_segment_size, self.discret_segment_size))

        self.map_targets = np.zeros(self.discret_segment_size * self.discret_segment_size).astype(np.float32)
        self.map_targets.resize((self.discret_segment_size, self.discret_segment_size))

        self.map_friends = copy.copy(self.map_targets)
        self.map_pose = copy.copy(self.map_targets)

        self.map_env = np.ones(self.discret_segment_size * self.discret_segment_size).astype(np.float32)
        self.map_env.resize((self.discret_segment_size, self.discret_segment_size))

    def get_segment_size(self):
        return self.segment_size

    def get_discret_segment_size(self):
        return self.discret_segment_size

    def get_nbr_cells(self):
        return self.map_targets.size

    def next_time(self):
        # The information evaporates
        self.map_targets -= 0.05 # 0.1
        self.map_friends -= 0.05 # 0.1
        self.map_pose -= 0.05
        # The idleness increase
        self.map_idleness += 1
        # Remove all but the target's pose
        self.map_targets[self.map_targets < 0] = 0.
        self.map_friends[self.map_friends < 0] = 0.
        self.map_pose[self.map_pose < 0] = 0.

    def pose_transform(self, x_in, y_in):
        ret = True
        x, y = int((x_in + self.map_size) * self.discretization), int((y_in + self.map_size) * self.discretization)
        if not 0 <= x < self.map_size * 2 * self.discretization or not 0 <= y < self.map_size * 2 * self.discretization:
            ret = False

        return ret, x, y

    def add_agent_moi(self, x, y):
        ret, x, y = self.pose_transform(x, y)
        if ret:
            self.map_pose[x, y] = 1.

    def add_agent_friend(self, x, y):
        ret, x, y = self.pose_transform(x, y)
        if ret:
            self.map_friends[x, y] = 1.

    def add_target(self, x, y):
        ret, x, y = self.pose_transform(x, y)
        if ret:
            self.map_targets[x, y] = 1.

    def get_max_idleness(self):
        return self.map_idleness.max()

    def observe(self, x, y, update_idleness=True):
        """

        :param x: coord x
        :param y: coord y
        :param update_idleness: False if we just evaluate without updating the idleness
        :return: sum of idleness
        """
        f = 0

        # Let reposition x and y [-map_size;map_size] into the map frame [0; map_size²]
        _, x, y = self.pose_transform(x, y)

        x_check, y_check = int(x), int(y)

        # Verify interval
        x_min_check = x_check - (self.obs_range * self.discretization)
        x_min_check = x_min_check if x_min_check > 0 else 0
        x_max_check = x_check + (self.obs_range * self.discretization)
        x_max_check = x_max_check if x_max_check < self.discret_segment_size else self.discret_segment_size
        y_min_check = y_check - (self.obs_range * self.discretization)
        y_min_check = y_min_check if y_min_check > 0 else 0
        y_max_check = y_check + (self.obs_range * self.discretization)
        y_max_check = y_max_check if y_max_check < self.discret_segment_size else self.discret_segment_size

        # Into int
        x_min_check = int(x_min_check)
        x_max_check = int(x_max_check)
        y_min_check = int(y_min_check)
        y_max_check = int(y_max_check)

        f = np.sum(self.map_idleness[x_min_check:x_max_check, y_min_check:y_max_check])
        if update_idleness:
            # Change all values to 0
            self.map_idleness[x_min_check:x_max_check, y_min_check:y_max_check] = 0

        return f

    def get_centered_map(self, x, y, mmap, default_outside_value=0.):

        _, x, y = self.pose_transform(x, y)
        windows_radius = int(self.com_range * 2 * self.discretization)

        # Get the windows size in the map frame
        x_min_w = x - windows_radius
        x_max_w = x + windows_radius
        y_min_w = y - windows_radius
        y_max_w = y + windows_radius

        # Check interval integrity with the original map
        x_min_check = x_min_w if x_min_w > 0 else 0
        y_min_check = y_min_w if y_min_w > 0 else 0
        x_max_check = x_max_w if x_max_w < self.discret_segment_size else self.discret_segment_size
        y_max_check = y_max_w if y_max_w < self.discret_segment_size else self.discret_segment_size

        # Create the window to return
        windows_map = np.ones((windows_radius * 2, windows_radius * 2)).astype(np.float32) * default_outside_value

        # Update window if sub is not empty
        if x_min_check < x_max_check and y_min_check < y_max_check:  # Else, we are outside the map boundaries
            for i in range(x_min_check, x_max_check):  # In the map frame
                for j in range(y_min_check, y_max_check):
                    windows_map[i - x_min_w, j - y_min_w] = mmap[i, j]  # To the window frame

        return windows_map

    def get_map(self, x_pose, y_pose, centered=True):
        x, y = copy.copy(x_pose), copy.copy(y_pose)
        norm_map_idleness = copy.copy(self.map_idleness)
        if norm_map_idleness.max() > 0:
            norm_map_idleness = norm_map_idleness / norm_map_idleness.max()
        if centered:
            obs_map = np.dstack((self.get_centered_map(x, y, copy.copy(self.map_env)),
                                 self.get_centered_map(x, y, copy.copy(self.map_targets)),
                                 self.get_centered_map(x, y, copy.copy(self.map_friends)),
                                 self.get_centered_map(x, y, copy.copy(norm_map_idleness)))).astype(np.float32)
        else:
            obs_map = np.dstack((self.map_pose, self.map_targets, self.map_friends, norm_map_idleness)).astype(np.float32)
        return obs_map

    def set_map(self, new_map):
        self.map_targets = new_map

    def update_from_map(self, other_map):
        self.map_idleness = np.where(self.map_idleness > other_map, other_map, self.map_idleness)

    def get_idleness_map(self):
        return copy.copy(self.map_idleness)
