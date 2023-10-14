import numpy as np

from model import *
from rigidbody import *
from sound import *

class RocketPod:
    def __init__(self, platform, rel_pos, num_rockets, cooldown=0.5, makesound=False):
        self.platform = platform
        self.rel_pos = rel_pos
        self.num_rockets = num_rockets
        self.cooldown = cooldown

        self.cooldown_timer = 0
        self.makesound = makesound
        # self.mass <--- implement later

    def shoot(self, bodies, target=None):
        if self.num_rockets and not self.cooldown_timer:
            init_pos = self.platform.pos + self.platform.orient[0] * self.rel_pos[0] + \
                       self.platform.orient[1] * self.rel_pos[1] + \
                       self.platform.orient[2] * self.rel_pos[2]
            
            init_vel = self.platform.vel            # m s-1
            init_accel = np.array([0, 0, 0])          # m s-2
            init_orient = self.platform.orient
            init_ang_vel = self.platform.ang_vel        # rad s-1
            init_ang_accel = np.array([0, 0, 0])      # rad s-2
            init_mass = 50                                # kg
            init_inertia = np.array([[500.0, 0.0, 0.0],
                                     [0.0, 500.0, 0.0],
                                     [0.0, 0.0, 1000.0]])     # kg m2
            max_thrust = 20e3                               # N
            throttle_range = [60, 100]                      # %
            throttle = 100                                  # %
            prop_mass = 40                                 # kg
            mass_flow = 3                                   # kg s-1

            Cds = np.array([0.3, 0.3, 0.05])
            Cdas = np.array([1, 1, 0.4])
            cross_sections = np.array([0.15, 0.15, 0.04])

            rocket_model = Model("rocket")
            init_CoM = np.array([0.0, 2.5, 0.0])

            new_rocket = Rocket(rocket_model, init_CoM,
                                init_pos, init_vel, init_accel,
                                init_orient, init_ang_vel, init_ang_accel,
                                init_mass, init_inertia,
                                max_thrust, throttle_range, throttle,
                                prop_mass, mass_flow, Cds, Cdas, cross_sections,
                                target)

            bodies.append(new_rocket)
        
            self.num_rockets -= 1
            self.cooldown_timer = self.cooldown
            if self.makesound:
                play_sfx("missile_launch", channel=4)
            
        elif not self.num_rockets and not self.cooldown_timer:
            self.cooldown_timer = self.cooldown
            if self.makesound:
                play_sfx("out", channel=4)

    def reload(self, num):
        self.num_rockets = num

    def update(self, dt):
        if self.cooldown_timer:
            self.cooldown_timer -= dt

        self.cooldown_timer = max(0, self.cooldown_timer)

