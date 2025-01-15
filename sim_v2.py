import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.animation import FuncAnimation
from itertools import count

from pid import rank_angles


def rotate_vector_around_axis(vector, axis, angle):
    axis = axis / np.linalg.norm(axis)
    x = axis[0]
    y = axis[1]
    z = axis[2]
    c = np.cos(angle)
    s = np.sin(angle)

    matrix = np.array([[x * x * (1 - c) + c,     x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                       [y * x * (1 - c) + z * s, y * y * (1 - c) + c,     y * z * (1 - c) - x * s],
                       [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, z * z * (1 - c) + c    ]])

    return matrix @ vector
def rotate_vector_around_axis_del_angle(vector, axis, angle):
    axis = axis / np.linalg.norm(axis)
    x = axis[0]
    y = axis[1]
    z = axis[2]
    c = np.cos(angle)
    s = np.sin(angle)

    matrix = np.array([[x * x * s - s,     x * y * s - z * c, x * z * s + y * c],
                       [x * y * s + z * c, y * y * s - s,     y * z * s - x * c],
                       [z * x * s - y * c, z * y * s + x * c, z * z * s - s    ]])
    return matrix @ vector

# Rocket Constants
SHAFT_CONNECTION_HEIGHT = 1.0  # Height difference of rocket centre of mass and shaft connection point
ROCKET_RADIUS = 1.0  # Distance from centre of rocket to fin connection point
FIN_FORCE_CONSTANT = 1.0  # Proportional to the surface area of the fin
ROCKET_MASS = 1.0
TIME_INTERVAL = 0.1


class Rocket:
    def __init__(self):
        self.torque = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.vector = SHAFT_CONNECTION_HEIGHT * np.array([0.0, 0.0, 1.0])

        self.lateral_forces = np.zeros(2)
        self.lateral_velocity = np.zeros(2)
        self.lateral_position = np.zeros(2)

    def vector_normalized(self):
        return self.vector / np.linalg.norm(self.vector)


class Fin:
    def __init__(self, position, displacement):
        self.position = position
        self.displacement = displacement

    def shaft_vector(self, rocket):
        return self.position - rocket.vector

    def fin_positive_normal(self, rocket):
        shaft_vec = self.shaft_vector(rocket)
        default_normal = np.cross(shaft_vec, rocket.vector)
        normal = rotate_vector_around_axis(default_normal, shaft_vec, self.displacement)
        return normal if normal[2] >= 0 else -normal

    def fin_force(self, rocket):
        normal = self.fin_positive_normal(rocket)
        magnitude = FIN_FORCE_CONSTANT * np.dot(normal, np.array([0, 0, 1]))
        return -normal * magnitude

    def rotate(self, axis, angle):
        self.position = rotate_vector_around_axis(self.position, axis, angle)


class RocketSimulation:
    def __init__(self):
        self.rocket = Rocket()

        fin_positions = [
            self.rocket.vector + ROCKET_RADIUS * np.array([1.0, 0.0, 0.0]),
            self.rocket.vector + ROCKET_RADIUS * np.array([0.0, 1.0, 0.0]),
            self.rocket.vector + ROCKET_RADIUS * np.array([-1.0, 0.0, 0.0]),
            self.rocket.vector + ROCKET_RADIUS * np.array([0.0, -1.0, 0.0])
        ]
        self.fins = [Fin(pos, 0.0) for pos in fin_positions]

    def calculate_total_torque(self):
        total_torque = np.zeros(3)
        for fin in self.fins:
            fin_force = fin.fin_force(self.rocket)
            total_torque += np.cross(fin_force, fin.position)
        return total_torque

    def calculate_angular_velocity(self):
        torque = self.calculate_total_torque()
        self.rocket.angular_velocity += torque * TIME_INTERVAL / ROCKET_MASS
        magnitude = np.linalg.norm(self.rocket.angular_velocity)
        if magnitude > 0:
            return self.rocket.angular_velocity / magnitude, magnitude
        return np.array([1.0, 0.0, 0.0]), 0.0


    def calculate_lateral_forces(self):
        total_lateral = np.zeros(2)
        for fin in self.fins:
            force = fin.fin_force(self.rocket)
            total_lateral += force[:2]  # Only take x and y components
        return total_lateral
    def calculate_lateral_velocity(self):
        self.rocket.lateral_velocity += self.rocket.lateral_forces * TIME_INTERVAL / ROCKET_MASS
        return self.rocket.lateral_velocity
    def calculate_lateral_position(self):
        self.rocket.lateral_position += self.rocket.lateral_velocity
        return self.rocket.lateral_position

    def set_fin_displacements(self, displacements):
        for fin, displacement in zip(self.fins, displacements):
            fin.displacement = displacement


    def update(self):
        axis, magnitude = self.calculate_angular_velocity()
        if magnitude > 0:
            # Rotate rocket vector
            self.rocket.vector = rotate_vector_around_axis(self.rocket.vector, axis, magnitude)
            # Rotate all fins
            for fin in self.fins:
                fin.rotate(axis, magnitude)
                #print(rank_angles(fin, self.rocket, 1)[0])
                fin.displacement = rank_angles(fin, self.rocket, 0.8)[0]

        self.rocket.lateral_forces = self.calculate_lateral_forces()
        self.rocket.lateral_velocity = self.calculate_lateral_velocity()
        self.rocket.lateral_position = self.calculate_lateral_position()
