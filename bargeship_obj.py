import numpy as np


class bargeship_obj(object):

    def __init__(self, ship_size, ship_com, mass, water_density):
        # ship base parameters
        self.ship_size = ship_size  # length, breadth, depth
        self.mass = mass  # in tons
        self.ship_com = ship_com
        self.z_pos = 0
        self.water_density = water_density

        self.angle = [0, 0, 0]
        self.moment = []

        self.vertices = self.generate_polygon()

    def generate_polygon(self):
        half_length = self.ship_size[0] / 2
        half_breadth = self.ship_size[1] / 2
        half_depth = self.ship_size[2] / 2

        vertices = np.array([
            [-half_length, -half_breadth, -half_depth],
            [half_length, -half_breadth, -half_depth],
            [half_length, half_breadth, -half_depth],
            [-half_length, half_breadth, -half_depth],
            [-half_length, -half_breadth, half_depth],
            [half_length, -half_breadth, half_depth],
            [half_length, half_breadth, half_depth],
            [-half_length, half_breadth, half_depth],
        ])
        # stern-port-bottom, bow-port-bottom, bow-starboard-bottom, stern-starboard-bottom, stern-port-top, bow-port-top, bow-starboard-top, stern-starboard-top

        return vertices
