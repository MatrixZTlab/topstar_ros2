from math import pi, fabs, sin, cos, atan2, acos
import numpy as np
from scipy.spatial.transform import Rotation

# DH_1 = [{"d": 0, "a": 0.044440, "alpha": -pi / 2, "offset": 0},
#           {"d": 0, "a": 0.442555, "alpha": 0, "offset": -pi / 2},
#           {"d": 0, "a": 0.034378, "alpha": -pi / 2, "offset": 0},
#           {"d": 0.4259830, "a": 0, "alpha": pi / 2, "offset": 0},
#           {"d": 0, "a": 0, "alpha": -pi / 2, "offset": 0},
#           {"d": 0.080, "a": 0, "alpha": 0, "offset": 0}]
# DH_2 = [{"d": 0, "a": 0.0463850, "alpha":-pi/2, "offset": 0},
#       {"d": 0, "a": 0.4423490, "alpha": 0, "offset": -pi/2},
#       {"d": 0, "a": 0.0357060, "alpha": -pi/2, "offset": 0},
#       {"d": 0.4254330, "a": 0, "alpha": pi/2, "offset": 0},
#       {"d": 0, "a": 0, "alpha": -pi/2, "offset": 0},
#       {"d": 0.080, "a": 0, "alpha": 0, "offset": 0}]

# Robots in the workstation
DH_1 = [{"d": 0, "a": 0.0450410, "alpha": -pi / 2, "offset": 0},
          {"d": 0, "a": 0.4418660, "alpha": 0, "offset": -pi / 2},
          {"d": 0, "a": 0.0356970, "alpha": -pi / 2, "offset": 0},
          {"d": 0.4255000, "a": 0, "alpha": pi / 2, "offset": 0},
          {"d": 0, "a": 0, "alpha": -pi / 2, "offset": 0},
          {"d": 0.080, "a": 0, "alpha": 0, "offset": 0}]
DH_2 = [{"d": 0, "a": 0.0465070, "alpha":-pi/2, "offset": 0},
      {"d": 0, "a": 0.4431460, "alpha": 0, "offset": -pi/2},
      {"d": 0, "a": 0.0353530, "alpha": -pi/2, "offset": 0},
      {"d": 0.4276450, "a": 0, "alpha": pi/2, "offset": 0},
      {"d": 0, "a": 0, "alpha": -pi/2, "offset": 0},
      {"d": 0.080, "a": 0, "alpha": 0, "offset": 0}]
limits = np.array([[-165, 165], [-90, 135], [-205, 65], [-185, 185], [-120, 120], [-360, 360]])/180*pi

tx_flangerot90_tip = np.identity(4)
tx_flangerot90_tip[:3, 3] = np.array([0, 0, 0.280])
tx_flangerot90_tip[:3, :3] = Rotation.from_euler('z', [np.pi / 2]).as_matrix()

tx_dummy = np.identity(4)

tx_flange_tip = tx_dummy
tx_tip_flange = np.linalg.inv(tx_flange_tip)

def dh_transform(theta, d, a, alpha):
    """Create a DH transformation matrix."""
    ct = cos(theta)
    st = sin(theta)
    ca = cos(alpha)
    sa = sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])


def check_joint_limits(joint_angles):
    for i in range(len(joint_angles)):
        if joint_angles[i] < limits[i][0] or joint_angles[i] > limits[i][1]:
            raise ValueError(
                f"Joint {i + 1} angle {joint_angles[i]:.2f} rad exceeds limits [{limits[i][0]:.2f}, {limits[i][1]:.2f}]")
    return True


def fkine(DH, joint_angles):
    check_joint_limits(joint_angles)
    T = np.eye(4)
    for i in range(6):
        theta = joint_angles[i] + DH[i]['offset']
        d = DH[i]['d']
        a = DH[i]['a']
        alpha = DH[i]['alpha']
        Ti = dh_transform(theta, d, a, alpha)
        T = T @ Ti
    return T


def ikine(DH, T, config="ruf", joint=None):
    """
    Analytic inverse kinematic solution
    ======   ==============================================
    Letter   Meaning
    ======   ==============================================
    l        Choose the left-handed configuration
    r        Choose the right-handed configuration
    u        Choose the elbow up configuration
    d        Choose the elbow down configuration
    n        Choose the wrist not-flipped configuration
    f        Choose the wrist flipped configuration
    ======   ==============================================
    joint: previous joint angles in rad
    """
    continuous = False
    j4 = 0
    max_step = 6000/60*2*pi/80
    if joint is not None:
        j4 = joint[3]
        continuous = True

    a1 = DH[0]["a"]
    a2 = DH[1]["a"]
    a3 = DH[2]["a"]
    d1 = DH[0]["d"]
    d3 = DH[2]["d"]
    d4 = DH[3]["d"]
    d6 = DH[5]["d"]

    R = T[:3, :3]
    P = T[:3, 3] - R[:, 2] * d6
    Px, Py, Pz = P
    Pz -= d1  # offset the pedestal height
    theta = np.zeros((6,))

    r = np.sqrt(Px ** 2 + Py ** 2)
    if "r" in config:
        theta[0] = atan2(Py, Px) + np.arcsin(d3 / r)
    elif "l" in config:
        theta[0] = atan2(Py, Px) + np.pi - np.arcsin(d3 / r)
    else:
        raise ValueError("bad configuration string")
    c1 = cos(theta[0])
    s1 = sin(theta[0])
    k = (Px*Px + Py*Py + Pz*Pz - d4*d4 - a3*a3 - a2*a2 + a1*a1 - 2*Px*a1*c1 - 2*Py*a1*s1) / (2*a2)
    k1 = d4*d4 + a3*a3 - k*k
    if k1 < 0:
        return "Out of reach"
    k1 = np.sqrt(k1)
    dir = 1
    if "l" in config:
        if "u" in config:
            dir = -1
    else:
        if "d" in config:
            dir = -1
    theta[2] = atan2(a3, d4) - atan2(k, dir*k1)
    c3 = cos(theta[2])
    s3 = sin(theta[2])

    u1 = a2 + a3 * c3 - d4 * s3
    v1 = -a3 * s3 - d4 * c3
    r1 = Px * c1 + Py * s1 - a1
    u2 = a3 * s3 + d4 * c3
    v2 = a2 + a3 * c3 - d4 * s3
    r2 = -Pz
    s2 = (u2 * r1 - u1 * r2) / (u2 * v1 - u1 * v2)
    c2 = (r1 - v1 * s2) / u1
    theta[1] = np.arctan2(s2, c2)

    s23 = sin(theta[1] + theta[2])
    c23 = cos(theta[1] + theta[2])

    theta[4] = acos(np.dot(np.array([-c1*s23, -s1*s23, -c23]), R[:, 2]))
    singular = False
    if theta[4] < 1e-6:
        singular = True
        theta[4] = 1e-6
    if ("l" in config and "f" in config) or ("r" in config and "n" in config):
        theta[4] = -theta[4]
    s5 = sin(theta[4])
    s4 = -R[0, 2]*s1 + R[1, 2]*c1
    c4 = np.dot(np.array([-c1*c23, -s1*c23, s23]), R[:, 2])
    repeat = 1
    while 0 < repeat < 3:
        s5 = sin(theta[4])
        if fabs(s4) < 1e-6 and fabs(c4) < 1e-6:
            theta[3] = 0
        else:
            theta[3] = atan2(s4*np.sign(s5), c4*np.sign(s5))
        if continuous and fabs(theta[3]-j4) > max_step:
            theta[4] = -theta[4]
            repeat += 1
        else:
            repeat = 0

    s6 = np.dot(np.array([c1*s23, s1*s23, c23]), R[:, 1]) / s5
    c6 = np.dot(np.array([-c1*s23, -s1*s23, -c23]), R[:, 0]) / s5
    theta[5] = atan2(s6, c6)
    if singular:
        c46 = np.dot(np.array([c1*c23, s1*c23, -s23]), R[:, 0])
        s46 = R[0, 0]*s1 - R[1, 0]*c1
        t46 = atan2(s46, c46)
        theta[5] = t46 - theta[3]

    for i in range(6):
        theta[i] -= DH[i]["offset"]

    if continuous:
        for i in [3, 5]:
            if theta[i] - joint[i] > pi:
                theta[i] -= 2*pi
            elif theta[i] - joint[i] < -pi:
                theta[i] += 2*pi

    return theta
