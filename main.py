import numpy as np
import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
import glfw
import time
import random
import keyboard as kbd
import json
import os

from rigidbody import *
from model import *
from graphics import *
from camera import *
from terrain import *
from ui import *
from scenery_objects import *
from sound import *
from alerts import *
from weapons import *
from propulsion import *
from city import *
from airframe import *

def read_industry():
    airframe_files = os.listdir("./data/industry/airframes/")
    engine_files = os.listdir("./data/industry/engines/")

    airframes = {}
    engines = {}

    # READ AIRFRAMES
    for airframe_file in airframe_files:
        file = open("./data/industry/airframes/" + airframe_file)
        data = json.load(file)
        airframes[data['name']] = Airframe(data['name'], data['manufacturer'],
                                           data['mass'], data['cd'], data['effectiveness'])
        print("Loading airframe:", data['name'])

    # READ ENGINES
    for engine_file in engine_files:
        file = open("./data/industry/engines/" + engine_file)
        data = json.load(file)
        
        intake = Intake(np.array([0, 0, -1]), 0.1)
        compressor = Compressor(data['compressor_coeff'], 100)
        turbine = Turbine(data['turbine_coeff'], data['max_rpm'])
        nozzle = Nozzle(data['thrust_coeff'])

        throttle_range = [0, 100] 
        efficiency = 0.6
        air_fuel_ratio = 2000 # this is an air volume vs. fuel MASS ratio, so it is not going to agree with anything familiar of course
        max_fuel_rate = data['fuel_consumption']
        APU_fuel_rate = data['APU_fuel_consumption']
        engines[data['name']] = Turbojet(intake, compressor, turbine, nozzle, efficiency, air_fuel_ratio, max_fuel_rate, throttle_range, APU_fuel_rate)
        print("Loading engine:", data['name'])

    return airframes, engines

def main():
    
    def window_resize(window, width, height):
        try:
            # glfw.get_framebuffer_size(window)
            glViewport(0, 0, width, height)
            glLoadIdentity()
            gluPerspective(fov, width/height, near_clip, far_clip)
            glTranslate(main_cam.pos[0], main_cam.pos[1], main_cam.pos[2])
            main_cam.orient = np.eye(3)
            main_cam.rotate([0, 180, 0])
        except ZeroDivisionError:
            # if the window is minimized it makes height = 0, but we don't need to update projection in that case anyway
            pass

    # READ INDUSTRY DATA
    airframes, engines = read_industry()
    player_airframe = airframes["LM Bombcat"]
    player_engine = engines["SE 7000"]
        
    # INIT VESSELS
    print("Initializing vessels...")
    init_pos = np.array([0.0, 2000.0, 0.0])         # m
    init_vel = np.array([0.0, 0.0, 100.0])          # m s-1
    init_accel = np.array([0.0, 0.0, 0.0])          # m s-2
    init_orient = np.array([[1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0],
                            [0.0, 0.0, 1.0]])
    init_ang_vel = np.array([0.0, 0.0, 0.0])        # rad s-1
    init_ang_accel = np.array([0.0, 0.0, 0.0])      # rad s-2
    init_mass = player_airframe.mass                                # kg
    init_inertia = np.array([[6000.0, 0.0, 0.0],
                             [0.0, 6000.0, 0.0],
                             [0.0, 0.0, 3000.0]])   # kg m2
    max_thrust = 5e3                               # N
    throttle_range = [0, 100]                       # %
    throttle = 100                                  # %
    prop_mass = 150                                 # kg
    mass_flow = 0.001                                # kg s-1

    plane_model = Model("plane_cockpit")
    init_CoM = np.array([0.0, 0.0, 2.0])

    cross_sections = np.array([8, 10, 2])            # m2
    Cds = np.array([0.6, 0.8, 0.1]) * player_airframe.Cd
    Cdas = np.array([8, 20, 45])
    angular_damping = np.array([0.4, 0.8, 0.8])
    Cl = 1.2
    lift_moment_arm = 0.15
    
    # aileron, elevator, rudder
    control_effectiveness = np.array([1.8, 1.8, 2.5]) * player_airframe.effectiveness

    rear_gear_moment = 1000                         # m N
    brake_force = 15e3                              # N

    rocket_pod = RocketPod(None, np.array([0.05, -0.1, 1.2]), 30)
    weapons = [rocket_pod]

    intake = Intake(np.array([0, 0, -1]), 0.1)
    compressor = Compressor(0.15, 100)
    turbine = Turbine(350, 7000)
    nozzle = Nozzle(5)

    efficiency = 0.6
    air_fuel_ratio = 2000 # this is an air volume vs. fuel MASS ratio, so it is not going to agree with anything familiar of course
    max_fuel_rate = 0.6
    APU_fuel_rate = 0.4
    jet = Turbojet(intake, compressor, turbine, nozzle, efficiency, air_fuel_ratio, max_fuel_rate, throttle_range, APU_fuel_rate)

    prop_mass = 150
    
    AP = Aircraft(plane_model, init_CoM,
                  init_pos, init_vel, init_accel,
                  init_orient, init_ang_vel, init_ang_accel,
                  init_mass, init_inertia, player_engine, prop_mass,
                  cross_sections, Cds, Cdas, angular_damping, Cl,
                  lift_moment_arm, control_effectiveness,
                  rear_gear_moment, brake_force, weapons)

    rocket_pod.platform = AP
    AP.set_thrust_percent(80)

    bodies = [AP]

    # SCENERY OBJECTS
    print("Initializing scenery objects...")
    pylon_model = Model("pylon")
    pylon1 = SceneryObject(pylon_model, np.array([10,0,500]))
    pylon2 = SceneryObject(pylon_model, np.array([-10,0,500]))

    # scenery_objects = [pylon1, pylon2]
    scenery_objects = []

    # cities
    city1 = City(np.array([0, 0, 0]), 1)
    city2 = City(np.array([10000, 0, -30000]), 0.5)
    city3 = City(np.array([50000, 0, 20000]), 1.4)

    cities = [city1, city2, city3]

    testructible = Testructible(np.array([150, 0, 0]))
    scenery_objects.append(testructible)

    # TERRAIN
    print("Initializing terrain...")
    floor = Flatland(0, Color(0.1, 0.8, 0.1))

    # MISC PHYSICS
    gravity = np.array([0.0, -9.81, 0])

    # GRAPHICS
    print("Initializing graphics (OpenGL, glfw)...")
    window_x, window_y = 1600, 900
    fov = 70
    near_clip = 0.1
    far_clip = 10e6
    
    glfw.init()
    window = glfw.create_window(window_x, window_y, "Kelaynak Flight Simulator", None, None)
    glfw.set_window_pos(window, 100, 100)
    glfw.make_context_current(window)
    glfw.set_window_size_callback(window, window_resize)

    gluPerspective(fov, window_x/window_y, near_clip, far_clip)
    glClearColor(0, 0, 0.3, 1)

    # SOUND
    print("Initializing sound (pygame.mixer)...")
    init_sound()

    # CAMERA
    cam_pos = np.array([0, 0, 0])
    cam_orient = np.array([[-1, 0, 0],
                           [0, 1, 0],
                           [0, 0, -1]])
    main_cam = Camera("main_cam", cam_pos, cam_orient, True)

    glRotate(-180, 0, 1, 0)    
    main_cam.lock_to_target(bodies[0])

    def move_cam(movement):
        main_cam.move(movement)

    def rotate_cam(rotation):
        main_cam.rotate(rotation)

    # CAMERA CONTROLS
    cam_pitch_up = "K"
    cam_pitch_dn = "I"
    cam_yaw_left = "J"
    cam_yaw_right = "L"
    cam_roll_cw = "O"
    cam_roll_ccw = "U"

    cam_move_fwd = "Y"
    cam_move_bck = "H"
    cam_move_left = "N"
    cam_move_right = "M"
    cam_move_up = "Y"
    cam_move_dn = "H"

    plane_pitch_up = "S"
    plane_pitch_dn = "W"
    plane_roll_ccw = "Q"
    plane_roll_cw = "E"
    plane_yaw_right = "D"
    plane_yaw_left = "A"
    plane_throttle_up = "Z"
    plane_throttle_dn = "X"

    shoot = "space"
    brake = "B"
    APU_start = "T"
    APU_stop = "G"

    metric_key = "M"
    imperial_key = "N"

    first_person_ui = True

    cam_speed = 100
    cam_rot_speed = 100

    play_sfx("turbojet_fan", -1, 1, 0)
    play_sfx("wind1", -1, 2, 0)
    play_sfx("rumble", -1, 3, 0)

    print("Starting...")
    dt = 0
    ctrl_state = [0, 0, 0]
    
    velocity_conversion_factor = 1
    altitude_conversion_factor = 1
    
    while not glfw.window_should_close(window):
        t_cycle_start = time.perf_counter()
        glfw.poll_events() 

        # CONTROLS
        if kbd.is_pressed(cam_move_fwd):
            move_cam([0, 0, cam_speed * dt])
        if kbd.is_pressed(cam_move_bck):
            move_cam([0, 0, -cam_speed * dt])
        if kbd.is_pressed(cam_move_up):
            move_cam([0, -cam_speed * dt, 0])
        if kbd.is_pressed(cam_move_dn):
            move_cam([0, cam_speed * dt, 0])
        if kbd.is_pressed(cam_move_right):
            move_cam([-cam_speed * dt, 0, 0])
        if kbd.is_pressed(cam_move_left):
            move_cam([cam_speed * dt, 0, 0])

        if kbd.is_pressed(cam_pitch_up):
            rotate_cam([cam_rot_speed * dt, 0, 0])
        if kbd.is_pressed(cam_pitch_dn):
            rotate_cam([-cam_rot_speed * dt, 0, 0])
        if kbd.is_pressed(cam_yaw_left):
            rotate_cam([0, cam_rot_speed * dt, 0])
        if kbd.is_pressed(cam_yaw_right):
            rotate_cam([0, -cam_rot_speed * dt, 0])
        if kbd.is_pressed(cam_roll_cw):
            rotate_cam([0, 0, cam_rot_speed * dt])
        if kbd.is_pressed(cam_roll_ccw):
            rotate_cam([0, 0, -cam_rot_speed * dt])

        if kbd.is_pressed(plane_pitch_up):
            ctrl_state[1] += 1 * dt
        elif kbd.is_pressed(plane_pitch_dn):
            ctrl_state[1] -= 1 * dt
        else:
            if abs(ctrl_state[1]) > 0.3:
                ctrl_state[1] *= 1 - 2 * dt
            else:
                ctrl_state[1] = 0

        if kbd.is_pressed(plane_roll_ccw):
            ctrl_state[0] += 1 * dt
        elif kbd.is_pressed(plane_roll_cw):
            ctrl_state[0] -= 1 * dt
        else:
            if abs(ctrl_state[0]) > 0.3:
                ctrl_state[0] *= 1 - 2 * dt
            else:
                ctrl_state[0] = 0

        if kbd.is_pressed(plane_yaw_right):
            ctrl_state[2] += 1 * dt
        elif kbd.is_pressed(plane_yaw_left):
            ctrl_state[2] -= 1 * dt
        else:
            if abs(ctrl_state[2]) > 0.3:
                ctrl_state[2] *= 1 - 2 * dt
            else:
                ctrl_state[2] = 0

        if kbd.is_pressed(plane_throttle_up):
            AP.update_throttle(30, dt)
        elif kbd.is_pressed(plane_throttle_dn):
            AP.update_throttle(-30, dt)

        if kbd.is_pressed(shoot):
            AP.weapons[0].shoot(bodies)

        if kbd.is_pressed(brake):
            AP.brake = 0.75
        else:
            AP.brake = 0

        for i in range(len(ctrl_state)):
            ctrl_state[i] = min(max(ctrl_state[i], -1), 1)

        AP.aileron(ctrl_state[0])
        AP.elevator(ctrl_state[1])
        AP.rudder(ctrl_state[2])

        if kbd.is_pressed(metric_key): # superior metric units for the superior people
            velocity_conversion_factor = 1
            altitude_conversion_factor = 1
        elif kbd.is_pressed(imperial_key): # inferior imperial units for Mars Climate Orbiter
            velocity_conversion_factor = 1.943844 # knots
            altitude_conversion_factor = 3.28084 # feet

        if kbd.is_pressed(APU_start):
            AP.engine.APU = True
        elif kbd.is_pressed(APU_stop):
            AP.engine.APU = False

        # PHYSICS

        for b in bodies:
            if isinstance(b, Rocket):
                b.apply_accel(gravity)
                b.apply_drag()
                b.apply_aero_torque()
                b.drain_fuel(dt)
                b.apply_thrust()
                b.update(dt)

                if b.pos[1] < floor.height:
                    bodies.remove(b)
                    del b

            elif isinstance(b, Aircraft):
                if b.pos[1] <= floor.height + 1:
                    if abs(b.orient[0][1]) < 0.3 and b.orient[1][1] > 0 and -0.1 < b.orient[2][1] < 0.6 \
                       and b.vel[1] >= -5: # horizontal and gentle enough
                        b.state = "LANDED"
                    else: # landed on its side or upside down, or too fast
                        b.state = "CRASHED"
                else:
                    b.state = "INFLIGHT"

        AP.engine.intake.air_intake_vector = -AP.orient[2]
            
        AP.drain_fuel(dt)
        AP.apply_aero_torque()
        AP.apply_angular_drag(dt)
        AP.apply_drag()
        AP.apply_lift()
        AP.apply_thrust(dt)
        AP.apply_brake()
        G = np.linalg.norm(AP.accel) / 10
        AP.apply_accel(gravity)
        AP.update_weapons(dt)
        AP.update(dt)

        AoA = np.arccos(max(min(np.dot(AP.vel, AP.orient[2]) / np.linalg.norm(AP.vel), 1), -1))
        AoA = np.rad2deg(AoA)

        # hit flat ground
        for b in bodies:
            if b.pos[1] < floor.height + 1:
                b.pos[1] = 1
                b.vel[1] = 0
                b.vel = b.vel - b.vel * 0.05 * dt

        main_cam.move_with_lock(dt)
        main_cam.rotate_with_lock(dt)

        # GRAPHICS
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        drawScene(main_cam, floor, bodies, cities, scenery_objects, ctrl_state, first_person_ui)
        
        alt_string = "Alt " + str(int(AP.pos[1] * altitude_conversion_factor))
        vel_string = "Vel " + str(int(np.linalg.norm(AP.vel) * velocity_conversion_factor))
        throttle_str = "Throttle " + str(int(AP.engine.throttle * 100))
        rpm_str = " RPM " + str(int(AP.engine.compressor.rpm))
        prop_str = "PROP " + str(int(AP.prop_mass))
        
        if AP.engine.APU:
            APU_str = "APU ACTV"
        else:
            APU_str = "APU DSBL"
            
        AoA_str = "AOA " + str(round(AoA, 2))
        G_str = "G " + str(round(G, 2))
        rockets_str = "RKT " + str(AP.weapons[0].num_rockets)

        magenta = Color(1, 0, 1)
        red = Color(1, 0, 0)
        
        AoA_color = magenta
        G_color = magenta
        vel_color = magenta
        alt_color = magenta
        throttle_color = magenta
        rpm_color = magenta
        APU_color = magenta
        rockets_color = magenta
        prop_color = magenta

        if np.linalg.norm(AP.vel) < 50 and AoA > 10:
            vel_color = red
            AoA_color = red
            if AP.engine.throttle < 100:
                throttle_color = red

        if G > 9:
            G_color = red

        if AP.pos[1] < 1000 and AP.vel[1] < 0 and AP.pos[1] / -AP.vel[1] < 3:
            alt_color = red

        if AP.weapons[0].num_rockets <= 3:
            rockets_color = red
        
        render_AN(alt_string, alt_color, [4, 5], main_cam, fpu=first_person_ui)
        render_AN(vel_string, vel_color, [-7, 5], main_cam, fpu=first_person_ui)
        render_AN(throttle_str, throttle_color, [-8, -4.5], main_cam, fpu=first_person_ui)
        render_AN(rpm_str, rpm_color, [-8, -5], main_cam, font_size=0.05, fpu=first_person_ui)
        render_AN(AoA_str, AoA_color, [-7, 4.5], main_cam, font_size=0.05, fpu=first_person_ui)
        render_AN(G_str, G_color, [4, 4.5], main_cam, font_size=0.05, fpu=first_person_ui)
        render_AN(rockets_str, rockets_color, [2, -5.5], main_cam, font_size=0.05, fpu=first_person_ui)
        render_AN(APU_str, APU_color, [-8, -5.4], main_cam, font_size=0.05, fpu=first_person_ui)
        render_AN(prop_str, prop_color, [-8, -5.8], main_cam, font_size=0.05, fpu=first_person_ui)
        
        glfw.swap_buffers(window)
        
        set_channel_volume(1, AP.engine.compressor.rpm/AP.engine.turbine.max_rpm * 0.5) # engine
        set_channel_volume(2, min(np.linalg.norm(AP.vel) / 500, 1) * 0.5) # airflow
        set_channel_volume(3, min(G / 10, 1) * 0.5) # airflow disturbance

        do_warnings(AP, AoA, G)

        dt = time.perf_counter() - t_cycle_start

    glfw.destroy_window(window)
    stop_channel(1)
    stop_channel(2)
    stop_channel(3)

main()
