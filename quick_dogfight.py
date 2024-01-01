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
from encounter import *

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
                                           data['mass'], data['cd'], data['effectiveness'],
                                           data['cargo_space'], data['price'])
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
        air_fuel_ratio = 20000 # this is an air volume vs. fuel MASS ratio, so it is not going to agree with anything familiar of course
        max_fuel_rate = data['fuel_consumption']
        APU_fuel_rate = data['APU_fuel_consumption']
        engines[data['name']] = Turbojet(data['name'], intake, compressor, turbine, nozzle, efficiency, air_fuel_ratio, max_fuel_rate, throttle_range, APU_fuel_rate, data['price'], data['manufacturer'])
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
    player_airframe = airframes["OSB Kirlangic"]
    player_engine = engines["APS 7K"]
        
    # INIT VESSELS
    print("Initializing vessels...")
    init_pos = np.array([0.0, 1000, 0.0])         # m
    init_vel = np.array([0.0, 0.0, 150.0])          # m s-1
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

    cargo_space = player_airframe.cargo_space

    rocket_pod = RocketPod(None, np.array([0.05, -0.1, 1.2]), 30, makesound=True)
    weapons = [rocket_pod]

    prop_mass = 150
    
    AP = Aircraft(plane_model, init_CoM,
                  init_pos, init_vel, init_accel,
                  init_orient, init_ang_vel, init_ang_accel,
                  init_mass, init_inertia, player_engine, prop_mass,
                  cross_sections, Cds, Cdas, angular_damping, Cl,
                  lift_moment_arm, control_effectiveness,
                  rear_gear_moment, brake_force, cargo_space, weapons)

    rocket_pod.platform = AP
    AP.set_thrust_percent(100)

    bodies = [AP]

    # SCENERY OBJECTS
    print("Initializing scenery objects...")
    pylon_model = Model("pylon")
    pylon1 = SceneryObject(pylon_model, np.array([10,0,500]))
    pylon2 = SceneryObject(pylon_model, np.array([-10,0,500]))

    # scenery_objects = [pylon1, pylon2]
    scenery_objects = []

    # cities
    city1 = City("Yazilikaya",

                 "A moderately sized agricultural city.\n\nHome to Ousteem Aerospace, which specializes in lightweight airframes.\nHome to AG Power Works, which specializes in heavy-duty propulsion.",
                 
                 [airframes["OSB Serce"], airframes["OSB Kirlangic"],
                  engines["APS 7K"], engines["APS 9K"]],

                {"Foodstuff": 8,
                 "Water": 0.7,
                 "Electronic Components": 300,
                 "Computers": 1500,
                 "Medicine": 380,
                 "Household Machinery": 400,
                 "Heavy Machinery": 120,
                 "Raw Ore": 0.1,
                 "Fuel": 25,
                 "Construction Materials": 20,
                 "Luxuries": 4000,
                 "Precious Metals": 7000},

                 np.array([0, 0, 0]), 1,

                 "death_mask")
    
    city2 = City("Numakawa",

                 "A small industrial city.\n\nHome to Matsuboshi, which specializes in heavy-lift airframes.\nHome to Kobesaki, which specializes in fuel-efficient propulsion.",
                 
                 [airframes["MSB Fuji"], airframes["MSB Kinki"],
                  engines["KS X7"], engines["KS X9"]],

                {"Foodstuff": 10,
                 "Water": 1,
                 "Electronic Components": 150,
                 "Computers": 800,
                 "Medicine": 410,
                 "Household Machinery": 300,
                 "Heavy Machinery": 80,
                 "Raw Ore": 5,
                 "Fuel": 60,
                 "Construction Materials": 18,
                 "Luxuries": 2500,
                 "Precious Metals": 15000},
                 
                 np.array([10000, 0, -30000]), 0.5,

                 "jade_empire")
    
    city3 = City("Meadowview",

                 "A large industrial metropolis.\n\nHome to Logheat Mardin, which specializes in serial production of airframes.\nHome to Special Electricity, which specializes in serial production of propulsion systems.",
                 
                 [airframes["LM Bombcat"], airframes["LM Wasp"],
                  engines["SE 7000"], engines["SE 9000"]],
                 
                 {"Foodstuff": 12,
                 "Water": 2,
                 "Electronic Components": 180,
                 "Computers": 1000,
                 "Medicine": 550,
                 "Household Machinery": 350,
                 "Heavy Machinery": 100,
                 "Raw Ore": 3,
                 "Fuel": 40,
                 "Construction Materials": 28,
                 "Luxuries": 2900,
                 "Precious Metals": 10000},
                 
                 np.array([40000, 0, 12000]), 1.4,

                 "trigger")

    cities = [city1, city2, city3]

    testructible = Testructible(np.array([150, 0, 0]))
    scenery_objects.append(testructible)

    # TERRAIN
    print("Initializing terrain...")
    floor = Flatland(0, Color(0.1, 0.8, 0.1))

    # MISC PHYSICS
    gravity = np.array([0.0, -9.81, 0])

    # SOUND
    print("Initializing sound (pygame.mixer)...")
    init_sound()

    # GRAPHICS
    print("Initializing graphics (OpenGL, glfw)...")
    window_x, window_y = 1600, 900
    fov = 70
    near_clip = 0.5
    far_clip = 1e6
    
    glfw.init()
    window = glfw.create_window(window_x, window_y, "Kelaynak 84", None, None)
    glfw.set_window_pos(window, 100, 100)
    glfw.make_context_current(window)
    glfw.set_window_size_callback(window, window_resize)

    gluPerspective(fov, window_x/window_y, near_clip, far_clip)
    glClearColor(0, 0, 0.3, 1)

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

    rwr_snd = None

    print("Starting...")
    print("= = = = = =\n")
    dt = 0
    ctrl_state = [0, 0, 0]
    
    velocity_conversion_factor = 1
    altitude_conversion_factor = 1

    pitch_trim = 0

    encounter_chance = 0.01
    current_encounter = None
    city_panel_shown = False
    hundred_cycle = 0
    AP_city = None
    player_money = 1000000
    player_cargo = {"Foodstuff": 0,
                    "Water": 0,
                    "Electronic Components": 0,
                    "Computers": 0,
                    "Medicine": 0,
                    "Household Machinery": 0,
                    "Heavy Machinery": 0,
                    "Raw Ore": 0,
                    "Fuel": 0,
                    "Construction Materials": 0,
                    "Luxuries": 0,
                    "Precious Metals": 0}

    game_running = True
    while game_running and not glfw.window_should_close(window):
        t_cycle_start = time.perf_counter()
        glfw.poll_events() 

        # CONTROLS

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
            if kbd.is_pressed("Shift"):
                pitch_trim += 0.2 * dt
            else:
                ctrl_state[1] += 1 * dt
        elif kbd.is_pressed(plane_pitch_dn):
            if kbd.is_pressed("Shift"):
                pitch_trim -= 0.2 * dt
            else:
                ctrl_state[1] -= 1 * dt
        else:
            if abs(ctrl_state[1] - pitch_trim) > 0.2:
                ctrl_state[1] = ctrl_state[1] - (ctrl_state[1] - pitch_trim) * dt
            else:
                ctrl_state[1] = pitch_trim

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
            if current_encounter:
                AP.weapons[0].shoot(bodies, current_encounter.enemy)
            else:
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
                b.update_trail()
                b.guidance(dt)
                b.apply_accel(gravity)
                b.apply_drag()
                b.apply_aero_torque()
                b.drain_fuel(dt)
                b.apply_thrust()
                b.update(dt)
                b.check_target(bodies)

                if b.pos[1] < floor.height:
                    bodies.remove(b)
                    del b

                elif np.linalg.norm(b.pos - AP.pos) > 30000:
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

        if np.linalg.norm(AP.vel):
            AoA = np.arccos(max(min(np.dot(AP.vel, AP.orient[2]) / np.linalg.norm(AP.vel), 1), -1))
            AoA = np.rad2deg(AoA)
        else:
            AoA = 0

        # hit flat ground
        for b in bodies:
            if b.pos[1] < floor.height + 1:
                b.pos[1] = 1
                b.vel[1] = 0
                b.vel = b.vel - b.vel * 0.05 * dt

        main_cam.move_with_lock(dt)
        main_cam.rotate_with_lock(dt)

        if not current_encounter == None:
            current_encounter.update(gravity, bodies, dt)
            rwr_snd = current_encounter.rwr(bodies)
            if current_encounter.state == "DOWN":
                bodies.remove(current_encounter.enemy)
                current_encounter = None

                if AP_city == None:
                    play_bgm("pluvious")
                else:
                    play_bgm(AP_city.bgm)

        if rwr_snd and (not (rwr_snd == "rwr_new" or rwr_snd == "rwr_lost")) and not get_channel_busy(6):
            play_sfx(rwr_snd, -1, 6)

        if rwr_snd and (rwr_snd == "rwr_new" or rwr_snd == "rwr_lost"):
            play_sfx(rwr_snd, 0, 8)
            
        elif not rwr_snd or rwr_snd == "rwr_new" or rwr_snd == "rwr_lost":
            stop_channel(6)
                
        # GRAPHICS
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        drawScene(main_cam, floor, bodies, cities, scenery_objects, ctrl_state, first_person_ui)
        
        alt_string = "Alt " + str(int(AP.pos[1] * altitude_conversion_factor))
        vel_string = "Vel " + str(int(np.linalg.norm(AP.vel) * velocity_conversion_factor))
        throttle_str = "Throttle " + str(int(AP.engine.throttle * 100))
        rpm_str = " RPM " + str(int(AP.engine.compressor.rpm))
        prop_str = "PROP " + str(int(AP.prop_mass))

        if AP_city:
            city_str = AP_city.name
        else:
            city_str = "Wilderness"
        
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
        city_color = magenta

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
        # render_AN(prop_str, prop_color, [-8, -5.8], main_cam, font_size=0.05, fpu=first_person_ui)
        render_AN(city_str, city_color, [0.5, -3.8], main_cam, font_size=0.08, fpu=first_person_ui)
        
        glfw.swap_buffers(window)
        
        set_channel_volume(1, AP.engine.compressor.rpm/AP.engine.turbine.max_rpm * 0.5) # engine
        set_channel_volume(2, min(np.linalg.norm(AP.vel) / 500, 1) * 0.5) # airflow
        set_channel_volume(3, min(G / 10, 1) * 0.5) # airflow disturbance

        do_warnings(AP, AoA, G)

        if AP.state == "LANDED" and np.linalg.norm(AP.vel) < 0.5 and AP_city and not city_panel_shown:
            render_AN("See the console window for city operations.", magenta, [-5, 2], main_cam, font_size=0.08, fpu=first_person_ui)
            glfw.swap_buffers(window)

            AP.vel = np.array([0, 0, 0])
            AP.ang_vel = np.array([0, 0, 0])
            AP.engine.compressor.rpm = 0

            # welcoming to the city
            print("")
            print("Welcome to " + AP_city.name + "!")
            print("")
            print(AP_city.desc)

            in_city = True
            while in_city:
                print("\nPlease make a choice: 1) Buy Commodities, 2) Sell Commodities, 3) Buy Aircraft Components, 9) Leave City")
                choice = input(" > ")

                if choice:
                    choice = int(choice)

                if choice == 1:
                    for key in AP_city.commodities:
                        print("\nYour cash:", player_money)
                        print("\nFree cargo space:", AP.cargo_space - sum(player_cargo.values()))
                        print("Commodity: " + key + "\nPrice: " + str(AP_city.commodities[key]))
                        buy_units = input("How many units would you like to buy? > ")

                        if buy_units:
                            buy_units = int(buy_units)
                        else:
                            buy_units = 0

                        if buy_units * AP_city.commodities[key] > player_money:
                            print("You do not have enough money to buy " + str(buy_units) + " units of " + key + "!")
                        elif buy_units + sum(player_cargo.values()) > AP.cargo_space:
                            print("You do not have enough cargo space to buy " + str(buy_units) + " units of " + key + "!")
                        else:
                            player_money -= buy_units * AP_city.commodities[key]
                            player_cargo[key] += buy_units

                elif choice == 2:
                    for key in player_cargo:
                        if player_cargo[key] > 0:
                            
                            print("\nCommodity: " + key + "\nPrice: " + str(AP_city.commodities[key]) + "\nUnits in hold: " + str(player_cargo[key]))
                            sell_units = int(input("How many units would you like to sell? > "))

                            if sell_units:
                                sell_units = int(sell_units)
                            else:
                                sell_units = 0

                            if sell_units > player_cargo[key]:
                                print("You do not have " + str(sell_units) + " units of " + key + " in your cargo hold!")
                            else:
                                player_money += sell_units * AP_city.commodities[key]
                                player_cargo[key] -= sell_units
                                print("Your cash:", player_money)

                elif choice == 3:
                    for part in AP_city.parts:
                        print("\nYour cash:", player_money)
                        print("Component: " + part.name + "\nManufacturer: " + part.manufacturer + "\nPrice: " + str(part.price))
                        buy_yn = input("Would you like to buy this component (y/N)? > ")

                        if buy_yn.lower() == "y":
                            if part.price <= player_money:
                                if part.type == "airframe":
                                    if sum(player_cargo.values()) > part.cargo_space:
                                        print("The new airframe does not have enough cargo space to hold your cargo!")
                                    else:
                                        player_airframe = part
                                        print("Purchased " + part.name + "!")
                                elif part.type == "engine":
                                    player_engine = part
                                    print("Purchased " + part.name + "!")

                                player_money -= part.price
                                
                                AP.mass = player_airframe.mass
                                AP.Cds = np.array([0.6, 0.8, 0.1]) * player_airframe.Cd
                                AP.control_effectiveness = np.array([1.8, 1.8, 2.5]) * player_airframe.effectiveness
                                AP.cargo_space = player_airframe.cargo_space

                                AP.engine = player_engine
                                
                            else:
                                print("You do not have enough money!")    

                elif choice == 9:
                    in_city = False
                
            city_panel_shown = True
            AP.weapons[0].reload(30)
            print("Please DO NOT CLOSE THIS WINDOW. You may return to the flight screen.")
            
        if city_panel_shown and AP.state != "LANDED":
            city_panel_shown = False

        if hundred_cycle > 99:
            hundred_cycle = 0

            # update which city the player is in
            city_found = False
            
            for c in cities:
                if np.linalg.norm(AP.pos + c.pos) < c.size * 1e4:
                    city_found = True
                    
                    if not c == AP_city:
                        AP_city = c
                        
                        if current_encounter == None:
                            play_bgm(c.bgm)    

                    break

            if not city_found:
                if not AP_city == None:
                    AP_city = None
                    if current_encounter == None:
                        play_bgm("pluvious")

        # ENCOUNTERS
        if AP.state == "INFLIGHT" and current_encounter == None:
            chance = random.uniform(0, 1)
            if chance < encounter_chance:
                current_encounter = Encounter(AP, "action", airframes, engines)
                bodies.append(current_encounter.enemy)
                play_bgm("massacre_machine")

        hundred_cycle += 1

        if AP.state == "CRASHED" or AP.hp <= 0:
            render_AN("GAME OVER", magenta, [-2, 2], main_cam, font_size=0.08, fpu=first_person_ui)
            glfw.swap_buffers(window)
            print("GAME OVER")
            input("Press Enter to end.")
            game_running = False

        dt = time.perf_counter() - t_cycle_start
        
    glfw.destroy_window(window)
    stop_channel(1)
    stop_channel(2)
    stop_channel(3)
    stop_channel(4)
    stop_channel(5)
    stop_channel(6)
    fade_out_bgm()

main()
