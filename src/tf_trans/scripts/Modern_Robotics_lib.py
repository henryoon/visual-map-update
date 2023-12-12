
# In modification_Donghun Lee

import math
import numpy as np

def quaternion_to_euler(r):
        (x, y, z, w) = (r[0], r[1], r[2], r[3])
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[2], r[1], r[0])
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return np.array([qx, qy, qz, qw])

def se3ToVec(se3mat):
    return np.r_[[se3mat[2][1], se3mat[0][2], se3mat[1][0]],
                [se3mat[0][3], se3mat[1][3], se3mat[2][3]]]


def VecTose3(V):
    return np.r_[np.c_[VecToso3([V[0], V[1], V[2]]), [V[3], V[4], V[5]]],
                np.zeros((1, 4))]


def VecToso3(omg):
    return np.array([[0, -omg[2], omg[1]],
                    [omg[2], 0, -omg[0]],
                    [-omg[1], omg[0], 0]])


def Adjoint(T):
    R, p = TransToRp(T)
    return np.r_[np.c_[R, np.zeros((3, 3))],
                np.c_[np.dot(VecToso3(p), R), R]]


def TransToRp(T):
    T = np.array(T)
    return T[0: 3, 0: 3], T[0: 3, 3]


def AxisAng3(expc3):
    return (Normalize(expc3), np.linalg.norm(expc3))


def so3ToVec(so3mat):
    return np.array([so3mat[2][1], so3mat[0][2], so3mat[1][0]])


def NearZero(z):
    return abs(z) < 1e-6


def MatrixLog6(T):
    R, p = TransToRp(T)
    omgmat = MatrixLog3(R)
    if np.array_equal(omgmat, np.zeros((3, 3))):
        return np.r_[np.c_[np.zeros((3, 3)),
                        [T[0][3], T[1][3], T[2][3]]],
                    [[0, 0, 0, 0]]]
    else:
        theta = np.arccos((np.trace(R) - 1) / 2.0)
        return np.r_[np.c_[omgmat,
                        np.dot(np.eye(3) - omgmat / 2.0 \
                                + (1.0 / theta - 1.0 / np.tan(theta / 2.0) / 2) \
                                * np.dot(omgmat, omgmat) / theta, [T[0][3],
                                                                    T[1][3],
                                                                    T[2][3]])],
                    [[0, 0, 0, 0]]]


def MatrixLog3(R):
    acosinput = (np.trace(R) - 1) / 2.0
    if acosinput >= 1:
        return np.zeros((3, 3))
    elif acosinput <= -1:
        if not NearZero(1 + R[2][2]):
            omg = (1.0 / np.sqrt(2 * (1 + R[2][2]))) \
                * np.array([R[0][2], R[1][2], 1 + R[2][2]])
        elif not NearZero(1 + R[1][1]):
            omg = (1.0 / np.sqrt(2 * (1 + R[1][1]))) \
                * np.array([R[0][1], 1 + R[1][1], R[2][1]])
        else:
            omg = (1.0 / np.sqrt(2 * (1 + R[0][0]))) \
                * np.array([1 + R[0][0], R[1][0], R[2][0]])
        return VecToso3(np.pi * omg)
    else:
        theta = np.arccos(acosinput)
        return theta / 2.0 / np.sin(theta) * (R - np.array(R).T)


def MatrixExp6(se3mat):
    se3mat = np.array(se3mat)
    omgtheta = so3ToVec(se3mat[0: 3, 0: 3])
    if NearZero(np.linalg.norm(omgtheta)):
        return np.r_[np.c_[np.eye(3), se3mat[0: 3, 3]], [[0, 0, 0, 1]]]
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = se3mat[0: 3, 0: 3] / theta
        return np.r_[np.c_[MatrixExp3(se3mat[0: 3, 0: 3]),
                        np.dot(np.eye(3) * theta \
                                + (1 - np.cos(theta)) * omgmat \
                                + (theta - np.sin(theta)) \
                                * np.dot(omgmat, omgmat),
                                se3mat[0: 3, 3]) / theta],
                    [[0, 0, 0, 1]]]


def MatrixExp3(so3mat):
    omgtheta = so3ToVec(so3mat)
    if NearZero(np.linalg.norm(omgtheta)):
        return np.eye(3)
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = so3mat / theta
        return np.eye(3) + np.sin(theta) * omgmat \
            + (1 - np.cos(theta)) * np.dot(omgmat, omgmat)


def Normalize(V):
    return V / np.linalg.norm(V)


def rotz(th):
    T = [[np.cos(th), -np.sin(th), 0],
        [np.sin(th), np.cos(th), 0],
        [0, 0, 1]]
    return T


def rotx(th):
    T = [[1, 0, 0],
        [0, np.cos(th), -np.sin(th)],
        [0, np.sin(th), np.cos(th)]]
    return T


def roty(th):
    T = [[np.cos(th), 0, np.sin(th)],
        [0, 1, 0],
        [-np.sin(th), 0, np.cos(th)]]
    return T

def eulertorotation(roll, pitch, yaw):
    r_matrix = [[round(math.cos(yaw)*math.cos(pitch),4),  round(math.cos(yaw)*math.sin(pitch)*math.sin(roll)-math.sin(yaw)*math.cos(roll),4),  round(math.cos(yaw)*math.sin(pitch)*math.cos(roll)+math.sin(yaw)*math.sin(roll),4)],
                [round(math.sin(yaw)*math.cos(pitch),4),  round(math.sin(yaw)*math.sin(pitch)*math.sin(roll)+math.cos(yaw)*math.cos(roll),4),  round(math.sin(yaw)*math.sin(pitch)*math.cos(roll)-math.cos(yaw)*math.sin(roll),4)],
                [round(-math.sin(pitch),4)             ,  round(math.cos(pitch)*math.sin(roll),4)                                           ,  round(math.cos(pitch)*math.cos(roll),4)]]
    return np.array(r_matrix)

def rotation_to_euler(mat):
    pitch = math.atan2(-mat[2][0], math.sqrt(mat[0][0] * mat[0][0] + mat[1][0] * mat[1][0]))
    yaw = math.atan2(mat[1][0] / math.cos(pitch), mat[0][0] / math.cos(pitch))
    roll = math.atan2(mat[2][1] / math.cos(pitch), mat[2][2] / math.cos(pitch))
    return roll, pitch, yaw