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
    angle_scores_positive = []
    for angle_pair in angle_scores:
        if angle_pair[1] >= 0: angle_scores_positive.append(angle_pair)
    angle_scores_sorted = sorted(angle_scores_positive, key=lambda x: x[1])
    if len(angle_scores_sorted) == 0: angle_scores_sorted.append(sorted(angle_scores, key=lambda x: x[1])[ANGLE_SAMPLE_SIZE - 1])
    # print(angle_scores_sorted)
    prop_int = np.rint(error(fin, rocket) * (len(angle_scores_sorted)) - 1).astype(int)
    print("prop_int = ", prop_int, "len(angle_scores_sorted) = ", len(angle_scores_sorted))
    print("Error: " , error(fin, rocket))
    print("rocket.vector.magnitude = ", np.linalg.norm(rocket.vector), "rocket.vector = ", rocket.vector)
    print("Angle score sorted: ", angle_scores_sorted)
    print("Angle score sorted at prop: " , angle_scores_sorted[prop_int])
    return angle_scores_sorted[prop_int]

def error(fin, rocket):
    dot = np.dot(rocket.vector, np.array([0,0,1]))
    print("dot = ", dot)
    return np.sqrt((1 - dot) / 2)







