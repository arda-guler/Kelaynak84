import random

from model import *
from graphics import Color

class SceneryObject:
    def __init__(self, model, pos, destructible=False, color=Color(0, 1, 1)):
        self.model = model
        self.pos = pos
        self.destructible = destructible
        self.color = color

class RandomBuilding(SceneryObject):
    def __init__(self, pos):
        super(RandomBuilding, self).__init__(Model("cube"), pos)
        rand_height = random.uniform(5, 70)
        rand_width = random.uniform(0.3, 0.7) * rand_height
        rand_depth = random.uniform(0.3, 0.7) * rand_height
        for idx_v, v in enumerate(self.model.vertices):
            self.model.vertices[idx_v] = np.array([self.model.vertices[idx_v][0] * rand_width,
                                                   self.model.vertices[idx_v][1] * rand_height,
                                                   self.model.vertices[idx_v][2] * rand_depth])

        self.bounding_box = np.array([rand_width, rand_height, rand_depth,
                                      rand_width, 0, rand_depth])

    def check_collision(self, other):
        # check height first, because it is the first thing that ought to be different if there is no collision
        # ...at least that's what I think.
        # anyway, if I do this, the if statement should go faster
        if self.pos[1] - self.bounding_box[1][1] <= other.pos[1] <= self.pos[1] + self.bounding_box[0][1] and \
           self.pos[0] - self.bounding_box[1][0] <= other.pos[1] <= self.pos[0] + self.bounding_box[0][0] and \
           self.pos[2] - self.bounding_box[1][2] <= other.pos[1] <= self.pos[2] + self.bounding_box[0][2]:

            return True

        return False

class Testructible(SceneryObject):
    def __init__(self, pos):
        super(Testructible, self).__init__(Model("cube"), pos, True)
        rand_height = random.uniform(5, 70)
        rand_width = random.uniform(0.3, 0.7) * rand_height
        rand_depth = random.uniform(0.3, 0.7) * rand_height
        for idx_v, v in enumerate(self.model.vertices):
            self.model.vertices[idx_v] = np.array([self.model.vertices[idx_v][0] * rand_width,
                                                   self.model.vertices[idx_v][1] * rand_height,
                                                   self.model.vertices[idx_v][2] * rand_depth])

        self.bounding_box = np.array([rand_width, rand_height, rand_depth,
                                      rand_width, 0, rand_depth])
        self.color = Color(1, 0, 0)

    def check_collision(self, other):
        # check height first, because it is the first thing that ought to be different if there is no collision
        # ...at least that's what I think.
        # anyway, if I do this, the if statement should go faster
        if self.pos[1] - self.bounding_box[1][1] <= other.pos[1] <= self.pos[1] + self.bounding_box[0][1] and \
           self.pos[0] - self.bounding_box[1][0] <= other.pos[1] <= self.pos[0] + self.bounding_box[0][0] and \
           self.pos[2] - self.bounding_box[1][2] <= other.pos[1] <= self.pos[2] + self.bounding_box[0][2]:

            return True

        return False
