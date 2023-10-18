import numpy as np

from sound import *

class Turbine:
    def __init__(self, conversion_rate, max_rpm):
        self.conversion_rate = conversion_rate
        self.max_rpm = max_rpm

    def compute_rpm(self, mass_flow, energy_utilization):
        return min((mass_flow * self.conversion_rate * energy_utilization), self.max_rpm)

class Compressor:
    def __init__(self, suction_rate, stall_flow):
        self.suction_rate = suction_rate
        self.stall_flow = stall_flow
        
        self.rpm = 0
        self.state = "OK"

    def compute_suction(self):
        return self.rpm * self.suction_rate

    def stall(self):
        play_sfx("compressor_stall", channel=5)
        self.state = "STALL"

class Intake:
    def __init__(self, intake_dir, area):
        self.intake_dir = intake_dir
        self.area = area
        self.air_intake_vector = intake_dir * area

    def compute_air_intake(self, vel):
        return max(-np.dot(vel, self.air_intake_vector), 0)

class Nozzle:
    def __init__(self, conversion_rate):
        self.conversion_rate = conversion_rate

    def compute_thrust(self, mass_flow, energy_utilization):
        return mass_flow * self.conversion_rate * energy_utilization

class Turbojet:
    def __init__(self, name, intake, compressor, turbine, nozzle, efficiency, air_fuel_ratio, max_fuel_rate,
                 throttle_range, APU_fuel_rate, price, manufacturer):
        self.name = name
        self.intake = intake
        self.compressor = compressor
        self.turbine = turbine
        self.nozzle = nozzle
        self.efficiency = efficiency
        self.air_fuel_ratio = air_fuel_ratio
        self.max_fuel_rate = max_fuel_rate
        self.throttle_range = throttle_range
        self.APU_fuel_rate = APU_fuel_rate
        self.price = price
        self.manufacturer = manufacturer
        self.type = "engine"
        
        self.APU = False
        self.throttle = 0
        self.thrust = 0
        self.fuel_rate = 0

    def compute_thrust(self, prop_mass, vel, dt):
        self.throttle = max(min(self.throttle, 1), 0)
        intake_air_momentum = self.intake.compute_air_intake(vel)

        if prop_mass <= 0:
            self.thrust = 0
            self.compressor.rpm = self.compressor.rpm * (1 - 0.2 * dt)
            return
        
        if not self.compressor.state == "STALL":
            intake_air_suction = self.compressor.compute_suction()
            intake_air = intake_air_momentum + intake_air_suction
            self.intake_air = intake_air
            self.intake_air_suction = intake_air_suction
            self.intake_air_momentum = intake_air_momentum

            if intake_air <= self.compressor.stall_flow and not self.APU:
                self.compressor.stall()
                self.thrust = 0
                return
            
            fuel_rate = min(intake_air / self.air_fuel_ratio, self.max_fuel_rate * self.throttle)
            mass_flow = fuel_rate + intake_air
            self.fuel_rate = fuel_rate

            if not fuel_rate:
                self.thrust = 0
                self.compressor.rpm = self.compressor.rpm * (1 - 0.2 * dt)
                return

            combustion_ratio = intake_air / fuel_rate
            energy_utilization = max(1 - abs(combustion_ratio - self.air_fuel_ratio) / self.air_fuel_ratio * 0.25, 0)
            
            turbine_rpm = self.turbine.compute_rpm(mass_flow, energy_utilization)
            self.compressor.rpm = self.compressor.rpm + (turbine_rpm - self.compressor.rpm) / 5 * dt

            self.thrust = self.nozzle.compute_thrust(mass_flow, energy_utilization)

            if self.APU:
                self.fuel_rate += self.APU_fuel_rate
        else:
            if self.APU or intake_air_momentum >= self.compressor.stall_flow:
                self.compressor.state = "OK"
                self.compressor.rpm = max(self.turbine.max_rpm * 0.2, self.compressor.rpm)
                self.fuel_rate = self.APU_fuel_rate
            else:
                self.compressor.rpm = self.compressor.rpm * (1 - 0.2 * dt)

