import numpy as np
#from sim_v2 import RocketSimulation, rotate_vector_around_axis_del_angle, Fin
#from sim_runner import sim

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

PROPORTIONAL_COEFFICIENT = 1.0
INTEGRAL_COEFFICIENT     = 1.0
DERIVATIVE_COEFFICIENT   = 1.0
ANGLE_SAMPLE_SIZE        = 50


angles_check = []
for i in range(ANGLE_SAMPLE_SIZE):
    angles_check.append( - np.pi / 2 + i * np.pi / (ANGLE_SAMPLE_SIZE - 1))

def fin_positive_normal_del_angle(fin, rocket, angle):
    shaft_vec = fin.shaft_vector(rocket)
    default_normal = np.cross(shaft_vec, rocket.vector)
    normal = rotate_vector_around_axis(default_normal, shaft_vec, angle)
    return normal if normal[2] >= 0 else -normal
def fin_torque_del_angle(fin, rocket, angle):
    return np.cross(fin_positive_normal_del_angle(fin, rocket, angle), fin.position)
def fin_torque_rating_del_angle(fin, rocket, angle):
    return -fin_torque_del_angle(fin, rocket, angle) @ np.cross(rocket.vector, np.array([0,0,1]))

def rank_angles(fin, rocket, prop):
    angle_scores = [(angle, fin_torque_rating_del_angle(fin, rocket, angle)) for angle in angles_check]
    # print(angle_scores)
    angle_scores_sorted = sorted(angle_scores, key=lambda x: x[1])
    # print(angle_scores_sorted)
    prop_int = np.rint((prop + 1)/2 * (ANGLE_SAMPLE_SIZE - 1)).astype(int)
    print("Angle score sorted:" , angle_scores_sorted[prop_int])
    print("the respective error" , error(fin, rocket))
    return angle_scores_sorted[prop_int]

def error(fin, rocket):
    theta = np.dot(rocket.vector, np.array([0,0,1]))
    theta = 1 / (1 + np.exp(-theta))
    cross_product = np.cross(rocket.vector, np.array([0,0,1]))
    sign = np.sign(cross_product[1])

    return sign * theta







