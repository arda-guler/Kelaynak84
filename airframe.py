class Airframe:
    def __init__(self, name, manufacturer, mass, Cd, effectiveness,
                 cargo_space, price):
        self.name = name
        self.manufacturer = manufacturer
        self.mass = mass
        self.Cd = Cd
        self.effectiveness = effectiveness
        self.cargo_space = cargo_space
        self.price = price
        self.type = "airframe"
