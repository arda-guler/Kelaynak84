import random

from rigidbody import *
from model import *
from weapons import *
from sound import *

class Encounter:
    def __init__(self, player, bgm, airframes, engines):
        self.player = player
        self.bgm = bgm
        self.generate_enemy(airframes, engines)
        self.state = "OK"

        self.rwr_new = False
        self.rwr_lost = False

    def generate_enemy(self, airframes, engines):
        player = self.player
        
        enemy_airframe_key = random.choice(list(airframes.keys()))
        enemy_engine_key = random.choice(list(engines.keys()))

        enemy_airframe = airframes[enemy_airframe_key]
        enemy_engine = engines[enemy_engine_key]
            
        # INIT VESSEL
        init_pos = player.pos + np.array([random.uniform(-1000, 1000), random.uniform(100, 500), random.uniform(-1000, 1000)])                                 # m

        player_rel_pos = (player.pos - init_pos)
        player_dist = np.linalg.norm(player.pos - init_pos)
        player_dir = player_rel_pos / player_dist
        
        init_vel = np.array([0, 0, 100])                # m s-1
        init_accel = np.array([0.0, 0.0, 0.0])          # m s-2
        init_orient = np.array([[1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0]])
        init_ang_vel = np.array([0.0, 0.0, 0.0])        # rad s-1
        init_ang_accel = np.array([0.0, 0.0, 0.0])      # rad s-2
        init_mass = enemy_airframe.mass                 # kg
        init_inertia = np.array([[6000.0, 0.0, 0.0],
                                 [0.0, 6000.0, 0.0],
                                 [0.0, 0.0, 3000.0]])   # kg m2
        max_thrust = 5e3                                # N
        throttle_range = [0, 100]                       # %
        throttle = 100                                  # %
        prop_mass = 150                                 # kg
        mass_flow = 0.001                               # kg s-1

        plane_model = Model("plane")
        init_CoM = np.array([0.0, 0.0, 2.0])

        cross_sections = np.array([8, 10, 2])           # m2
        Cds = np.array([0.6, 0.8, 0.1]) * enemy_airframe.Cd
        Cdas = np.array([8, 20, 45])
        angular_damping = np.array([0.4, 0.8, 0.8])
        Cl = 1.2
        lift_moment_arm = 0.15
        
        # aileron, elevator, rudder
        control_effectiveness = np.array([1.8, 1.8, 2.5]) * enemy_airframe.effectiveness

        rear_gear_moment = 1000                         # m N
        brake_force = 15e3                              # N

        cargo_space = enemy_airframe.cargo_space

        enemy_rocket_pod = RocketPod(None, np.array([0.05, -0.1, 1.2]), 30)
        weapons = [enemy_rocket_pod]

        prop_mass = 150
        
        self.enemy = Aircraft(plane_model, init_CoM,
                      init_pos, init_vel, init_accel,
                      init_orient, init_ang_vel, init_ang_accel,
                      init_mass, init_inertia, enemy_engine, prop_mass,
                      cross_sections, Cds, Cdas, angular_damping, Cl,
                      lift_moment_arm, control_effectiveness,
                      rear_gear_moment, brake_force, cargo_space, weapons)

        enemy_rocket_pod.platform = self.enemy
        self.enemy.engine.APU = True
        self.enemy.set_thrust_percent(100)

    def update(self, gravity, bodies, dt):
        player = self.player
        self.enemy.engine.throttle = 1

        player_actual_rel_pos = (player.pos - self.enemy.pos)
        player_actual_dist = np.linalg.norm(player.pos - self.enemy.pos)

        # aimpoint position
        player_rel_pos = (player.pos - self.enemy.pos) + player.vel * player_actual_dist / 1000
        player_dist = np.linalg.norm(player_rel_pos)
        player_dir = player_rel_pos / player_dist

        # AUTOPILOT

        # pull up!
        if self.enemy.pos[1] < 300 and np.dot(self.enemy.orient[2], np.array([0, -1, 0]) > -0.1):
            max_orient_1 = np.linalg.norm( np.array([0, 1, 0]) - np.dot(self.enemy.orient[2], np.array([0, 1, 0])) * self.enemy.orient[2] )
            if np.dot(self.enemy.orient[1], np.array([0, 1, 0])) < max_orient_1 * 0.8:
                if np.dot(self.enemy.orient[0], np.array([0, 1, 0])) < 0:
                    self.enemy.aileron(-0.8)
                else:
                    self.enemy.aileron(0.8)

            if np.dot(self.enemy.orient[1], np.array([0, 1, 0])) > 0:
                self.enemy.elevator(0.8)

        # dogfight
        else:
            max_orient_1 = np.linalg.norm( player_dir - np.dot(self.enemy.orient[2], player_dir) * self.enemy.orient[2] )
            # lift vector not aligned with player?
            if np.dot(self.enemy.orient[1], player_dir) < max_orient_1 * 0.8:

                # should I roll clockwise?
                if np.dot(self.enemy.orient[0], player_dir) > 0:
                    aileron_amount = np.dot(self.enemy.orient[0], player_dir)
                    self.enemy.aileron(0.8 * aileron_amount)
                else:
                    aileron_amount = -np.dot(self.enemy.orient[0], player_dir)
                    self.enemy.aileron(-0.8 * aileron_amount)

            # should I pitch?
            if np.linalg.norm(self.enemy.vel) < 100:
                elevator_scaler = (np.linalg.norm(self.enemy.vel) - 100) / 100
            else:
                elevator_scaler = 1
            elevator_amount = -(np.dot(self.enemy.orient[2], player_dir) - 1)
            self.enemy.elevator(elevator_amount)

            if np.dot(self.enemy.orient[1], player_dir) > 0:
                if np.dot(self.enemy.orient[2], player_dir) < 0.94:
                    self.enemy.elevator(0.8)

                else:
                    self.enemy.elevator(0.3)
                    self.enemy.weapons[0].shoot(bodies, self.player)

        self.enemy.engine.intake.air_intake_vector = -self.enemy.orient[2]
            
        self.enemy.drain_fuel(dt)
        self.enemy.apply_aero_torque()
        self.enemy.apply_angular_drag(dt)
        self.enemy.apply_drag()
        self.enemy.apply_lift()
        self.enemy.apply_thrust(dt)
        self.enemy.apply_brake()
        self.enemy.apply_accel(gravity)
        self.enemy.update_weapons(dt)
        self.enemy.update(dt)

        if self.enemy.pos[1] < 5 or self.enemy.hp <= 0:
            self.state = "DOWN"
            play_sfx("explosion", channel=6)

    def rwr(self, bodies):
        if not self.rwr_new:
            self.rwr_new = True
            return "rwr_new"

        if self.state == "DOWN":
            self.rwr_lost = True
            return "rwr_lost"

        player = self.player
        player_rel_pos = (player.pos - self.enemy.pos)
        player_dist = np.linalg.norm(player.pos - self.enemy.pos)
        player_dir = player_rel_pos / player_dist

        for b in bodies:
            if isinstance(b, Rocket) and b.target == player and np.dot(b.orient[2], (player.pos - b.pos)) > 0 and np.dot(b.vel, (player.pos - b.pos)) > 0:
                return "rwr_incoming"
        
        if np.dot(self.enemy.orient[1], player_dir) > 0:
                if np.dot(self.enemy.orient[2], player_dir) > 0.8:
                    return "rwr_targeted"

        return None

