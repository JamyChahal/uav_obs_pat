import math


class PotentialField():

    def __init__(self, do1, do2, do3, do4, dr1, dr2):
        self.potential_field = []

        # Parameters for A-CMOMMT
        # WALKER
        self.do1 = do1  # Repulsion until this distance
        self.do2 = do2  # Desired distance : Maximum magnitude = 1
        self.do3 = do3  # Last desired distance : Magnitude = 1. Start decrease
        self.do4 = do4  # Magnitude 0
        # ROBOT
        self.dr1 = dr1  # Extreme repulsion until this distance
        self.dr2 = dr2  # Slow repulsion until 0 at this distance

        self.obs_radius = self.do4


        # Magnitude functions
        self.f1a = (0 - (-1)) / float(self.do1 - 0)
        self.f1b = -1
        self.f2a = (1 - 0) / float(self.do2 - self.do1)
        self.f2b = 0 - self.f2a * self.do1  # b = yA - a*xA
        self.f3a = 0
        self.f3b = 1
        self.f4a = (-1) / float((self.do4 - self.do3))
        self.f4b = 1 - self.f4a * self.do3

        self.fr1a = 1 / float(self.dr2 - self.dr1)
        self.fr1b = -1 - self.fr1a * self.dr1

        self.x, self.y = 0, 0

        self.was_empty = True

    def set_pose(self, x, y):
        self.x, self.y = x, y

    def add_friend(self, x, y):
        dist = self.get_ground_distance(x, y)
        if dist < self.dr2:
            magnitude = self.get_friend_magnitude(dist)
            orientation = math.atan2(y - self.y, x - self.x)
            self.potential_field.append([orientation, magnitude])

    def add_target(self, x, y):
        dist = self.get_ground_distance(x, y)
        if dist <= self.obs_radius:
            magnitude = self.get_target_magnitude(dist)
            orientation = math.atan2(y - self.y, x - self.x)
            self.potential_field.append([orientation, magnitude])

    def reset(self):
        if len(self.potential_field) > 0:
            self.was_empty = False
        else:
            self.was_empty = True
        self.potential_field = []

    def is_empty(self):
        if len(self.potential_field) == 0:
            return True
        else:
            return False

    def is_still_empty(self):
        if self.was_empty and len(self.potential_field) == 0:
            return True
        else:
            return False

    def get_resultante(self):
        if not self.is_empty():
            angle, norm = self.circular_mean()
            return angle, norm
        else:
            print("get_resultante should not be called !")
            return 0, 0

    def circular_mean(self):
        x = y = 0
        for i in range(0, len(self.potential_field)):
            x += math.cos(self.potential_field[i][0]) * self.potential_field[i][1]
            y += math.sin(self.potential_field[i][0]) * self.potential_field[i][1]

        mean_angle = math.atan2(y, x)
        norm = math.sqrt(math.pow(x, 2) + math.pow(y, 2))


        # print("Global target : " + str(x) + " : " + str(y))
        # print("Angle " + str(mean_angle) + " norm " + str(norm))

        return mean_angle, norm

    def get_target_magnitude(self, distance):
        m = 0
        if distance <= self.do1:
            m = self.f1a * distance + self.f1b
        if self.do1 < distance <= self.do2:
            m = self.f2a * distance + self.f2b
        if self.do2 < distance <= self.do3:
            m = self.f3a * distance + self.f3b
        if self.do3 < distance <= self.do4:
            m = self.f4a * distance + self.f4b
        if distance > self.do4:
            m = 0

        # print("Distance : " + str(distance) + " magnitude : " + str(m))
        return m

    def get_friend_magnitude(self, distance):
        m = 0
        if distance <= self.dr1:
            m = -1
        if self.dr1 < distance <= self.dr2:
            m = self.fr1a * distance + self.fr1b
        if distance > self.dr2:
            m = 0

        return m

    def get_ground_distance(self, x, y):
        return math.sqrt(math.pow(self.x - x, 2) + math.pow(self.y - y, 2))

    def get_debug_info(self):
        if self.is_empty():
            print("PF is empty")
        else:
            for pf in self.potential_field:
                print("NORM:" + str(pf[1]) + " ORIENT:" + str((pf[0])))