import random
import numpy as np

from scenery_objects import *

class City:
    def __init__(self, pos, size):
        self.pos = pos
        self.size = size
        self.buildings = self.generate()

    def generate(self):
        buildings = []
        Nx = int(100 * self.size)
        Nz = int(100 * self.size)
        chance = 0.01
        building_spacing_x = 50
        building_spacing_z = 50

        building_area_corner_x = Nx / 2 * building_spacing_x + self.pos[0]
        building_area_corner_z = Nz / 2 * building_spacing_z + self.pos[2]

        for idx_x in range(Nx):
            for idx_z in range(Nz):
                if random.uniform(0, 1) < chance:
                    c_x = -building_area_corner_x + idx_x * building_spacing_x
                    c_z = -building_area_corner_z + idx_z * building_spacing_z
                    new_pos = np.array([c_x, 0, c_z])
                    new_building = RandomBuilding(new_pos)
                    buildings.append(new_building)

        return buildings
