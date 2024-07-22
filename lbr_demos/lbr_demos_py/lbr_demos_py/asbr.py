'''Collection of functions/types from the ASBR class taken in Spring 2022

Most code is derived from the Matlab code that was written during that class,
and is broadly attributable to Dr Alambeigi (who taught the course).
'''

import numpy as np
import quaternion  # adds quaternion type to numpy namespace
from math import sin, cos, atan2, asin, pi
from scipy import linalg
import geometry_msgs.msg  # For conversion to/from message types

# numpy also offers versions of these
def deg2rad(d):
    if isinstance(d, np.ndarray):
        return np.deg2rad(d)
    elif isinstance(d, list):
        return [di / 180 * pi for di in d]
    elif isinstance(d, tuple):
        return tuple([di / 180 * pi for di in d])
    else:
        return d / 180 * pi
def rad2deg(r):
    if isinstance(r, np.ndarray):
        return np.rad2deg(r)
    elif isinstance(r, list):
        return [ri / pi * 180 for ri in r]
    elif isinstance(r, tuple):
        return tuple([ri / pi * 180 for ri in r])
    else:
        return r / pi * 180

def is_near(a, b, close_enough=1e-14):
    return abs(a - b) < close_enough

def is_eye(a, close_enough=1e-14):
    return np.allclose(a, np.eye(a.shape[0]), atol=close_enough)

def skewsym(v):
    if v.size == 3:
        return np.array([[    0, -v[2],  v[1]],
                         [ v[2],     0, -v[0]],
                         [-v[1],  v[0],     0]])
    elif v.size == 6:
        return np.array([[    0, -v[2],  v[1], v[3]],
                         [ v[2],     0, -v[0], v[4]],
                         [-v[1],  v[0],     0, v[5]],
                         [    0,     0,     0,    0]])
    else:
        raise Exception('Not implemented for this shape of input: ' + str(v.size))



class Degrees():
    def __init__(self, x_):
        self.x = x_
    def __str__(self):
        return str(self.x)
    @property
    def deg(self):
        return self.x
    @deg.setter
    def deg(self, val):
        self.x = val
    @property
    def rad(self):
        return deg2rad(self.x)
    @rad.setter
    def rad(self, val):
        self.x = rad2deg(val)

# Discovered Rotation is in scipy.spatial.transform.
#from scipy.spatial.transform import Rotation
# But nothing else is... so we still have to write a bunch of stuff. Maybe just redo scipy work.
class Rotation():
    def __init__(self, init_value=None):
        if init_value is None:
            self.m = np.eye(3)
        elif isinstance(init_value, np.quaternion):
            self.m = quaternion.as_rotation_matrix(init_value)
        elif isinstance(init_value, geometry_msgs.msg.Quaternion):
            self.m = quaternion.as_rotation_matrix(np.quaternion(init_value.w, init_value.x, init_value.y, init_value.z))
        elif isinstance(init_value, np.ndarray):
            assert(len(init_value.shape) == 2)
            assert(init_value.shape[0] == 3)
            assert(init_value.shape[1] == 3)
            self.m = init_value
        elif isinstance(init_value, Rotation):
            self.m = np.copy(init_value.m)
        else:
            raise Exception('Dont know how to init Rotation from this type')
    def __str__(self):
        return str(self.m)
    def __mul__(self, other):
        return Rotation(np.matmul(self.m, other.m))
    @staticmethod
    def eye():
        return Rotation(None)
    @staticmethod
    def from_zyx(euler, degrees=False):
        #euler = [alpha, beta, gamma]
        #1: Rot about fixed x (gamma, yaw), 2: Rot about fixed y (beta, pitch), 3: Rot about fixed z (alpha, roll)
        if degrees:
            eulerrad = deg2rad(euler)
        else:
            eulerrad = euler
        # Unfortunately, quaternion module doesn't support various euler conventions
        # So I implement via ASBR's rpy2rot function from Matlab.
        alpha = eulerrad[0]; beta = eulerrad[1]; gamma = eulerrad[2]  # reverses, bc formula is rpy
        ca = cos(alpha); sa = sin(alpha)
        cb = cos(beta);  sb = sin(beta)
        cg = cos(gamma); sg = sin(gamma)
        return Rotation(np.array([[ca*cb,  ca*sb*sg-sa*cg,  ca*sb*cg+sa*sg],
                                  [sa*cb,  sa*sb*sg+ca*cg,  sa*sb*cg-ca*sg],
                                  [-sb,    cb*sg,           cb*cg]]))
    @staticmethod
    def from_zyx_degrees(euler):
        # Just forwarding, for backward compatibility.
        return Rotation.from_zyx(euler, degrees=True)
    @staticmethod
    def from_ABC(abc, degrees=False):
        # An alias for zyx (Jeff)
        # Not sure this is exactly as zyx. It's more like x"(C).y'(B).z(A) i.e. 1: about z (A), 2: about y' (B), 3: about x" (C)
        # [A,B,C]
        # The trick is, x"(C).y'(B).z(A) = z(A).y(B).x(C)
        return Rotation.from_zyx(abc, degrees=degrees)
        # Alternative implementation via quaternions:
        #  https://doc.rc-visard.com/v21.07/en/pose_format_kuka.html
        #if degrees:
        #    (Ar, Br, Cr) = deg2rad(abc)
        #else:
        #    (Ar, Br, Cr) = abc
        #x = cos(Ar/2) * cos(Br/2) * sin(Cr/2) - sin(Ar/2) * sin(Br/2) * cos(Cr/2) 
        #y = cos(Ar/2) * sin(Br/2) * cos(Cr/2) + sin(Ar/2) * cos(Br/2) * sin(Cr/2)
        #z = sin(Ar/2) * cos(Br/2) * cos(Cr/2) - cos(Ar/2) * sin(Br/2) * sin(Cr/2) 
        #w = cos(Ar/2) * cos(Br/2) * cos(Cr/2) + sin(Ar/2) * sin(Br/2) * sin(Cr/2)
        #return Rotation(np.quaternion(w, x, y, z))
    @staticmethod
    def from_ABC_degrees(abc_deg):
        # Just forwarding, for backward compatibility.
        return Rotation.from_ABC(abc_deg, degrees=True)
    @staticmethod
    def from_rpy(rpy, degrees=False):
        #Omid: alias as ZYX, [Roll, Pitch, Yaw]
        return Rotation.from_zyx(rpy, degrees=degrees)
    def is_eye(self):
        return is_eye(self.m)
    def as_zyx(self, degrees=False):
        # TODO: what is the range of these? I think pitch is -pi/2 to +pi/2. Others are... not sure.
        if self.is_eye():
            return np.zeros([3, 1])
        else:
            alpha = atan2(self.m[1,0], self.m[0,0])
            beta = atan2(-self.m[2,0], np.sqrt(self.m[2,1]**2 + self.m[2,2]**2))
            gamma = atan2(self.m[2,1], self.m[2,2])
            ans = np.array([alpha, beta, gamma])
            return np.rad2deg(ans) if degrees else ans
    def as_ABC(self, degrees=False):
        # Just an alias for zyx
        return self.as_zyx(degrees=degrees)
        # Alternative implementation, seems to be the same, but worse numerical stability (?), from:
        #   https://doc.rc-visard.com/v21.07/en/pose_format_kuka.html
        #q = self.as_quat()  # The formulas I have use a quaternion, so need that first.
        #A = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        #B = asin(2 * (q.w * q.y - q.z * q.x))
        #C = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2))
        #ans = np.array([A, B, C]).T
        #return np.rad2deg(ans) if degrees else ans
    def as_rpy(self, degrees=False):
        # Same as ZYX, but presented in different order. (Jeff; not sure is true)
        zyx = self.as_zyx(degrees)
        return zyx
    def as_quat(self):
        return quaternion.from_rotation_matrix(self.m)
    def as_geometry_orientation(self):
        q = self.as_quat()
        return geometry_msgs.msg.Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)

class Translation():
    def __init__(self, init_xyz=None):
        if init_xyz is not None:
            if isinstance(init_xyz, geometry_msgs.msg.Point):
                self.m = np.array([init_xyz.x, init_xyz.y, init_xyz.z])
            elif isinstance(init_xyz, geometry_msgs.msg.Vector3):
                self.m = np.array([init_xyz.x, init_xyz.y, init_xyz.z])
            elif isinstance(init_xyz, HPoint):
                self.m = init_xyz.xyz
            elif isinstance(init_xyz, Translation):
                self.m = np.copy(init_xyz.m)
            else:
                self.m = np.squeeze(np.array(init_xyz))
                assert(self.m.size == 3)
        else:
            self.m = np.zeros([3,1])
    def __str__(self):
        return str(self.m)
    def __add__(self, other):
        return Translation(self.m + other.m)
    @property
    def x(self):
        return self.m[0]
    @x.setter
    def x(self, val):
        self.m[0] = val
    @property
    def y(self):
        return self.m[1]
    @y.setter
    def y(self, val):
        self.m[1] = val
    @property
    def z(self):
        return self.m[2]
    @z.setter
    def z(self, val):
        self.m[2] = val
    def distance(self):
        return np.linalg.norm(self.m)
    def scale(self, factor):
        self.m = self.m * factor
    def convert_m_to_mm(self):
        self.scale(1000.)
    def convert_mm_to_m(self):
        self.scale(0.001)
    def scaled(self, factor):
        return Translation(self.m * factor)
    def scaled_m_to_mm(self):
        return self.scaled(1000.)
    def scaled_mm_to_m(self):
        return self.scaled(0.001)
    def as_geometry_point(self):
        return geometry_msgs.msg.Point(x=self.x, y=self.y, z=self.z)

class Transformation:
    def __init__(self, *args):
        if len(args) == 1:
            init_value = args[0]
            if isinstance(init_value, np.ndarray):
                assert(len(init_value.shape) == 2)
                assert(init_value.shape[0] == 4)
                assert(init_value.shape[1] == 4)
                assert(np.allclose(init_value[3, :], [0, 0, 0, 1]))
                self.m = init_value
            elif isinstance(init_value, geometry_msgs.msg.Pose):
                self.m = np.eye(4)
                self.rotation = Rotation(init_value.orientation)
                self.translation = Translation(init_value.position)
            elif isinstance(args[0], Translation):
                self.m = np.eye(4)
                self.translation = args[0]
            else:
                raise Exception('Dont know how to init Transformation from this init_value type')
        elif len(args) == 2:
            if isinstance(args[0], Translation) and isinstance(args[1], Rotation):
                self.m = np.eye(4)
                self.translation = args[0]
                self.rotation = args[1]
            else:
                raise Exception('Dont know how to init Transformation from these args of type {} and {}'.format(type(args[0]), type(args[1])))
        else:
            self.m = np.eye(4)
    def __str__(self):
        return str(self.m)
    def __mul__(self, other):
        if isinstance(other, Transformation):
            return Transformation(self.m @ other.m)
        else:
            raise TypeError('Invalid type of multiplication argument for Transformation: ' + str(type(other)))
    def rot(self):
        assert(np.allclose(self.m[3, :], [0, 0, 0, 1]))
        #return self.m[0:3, 0:3]
        #return Rotation.from_matrix(self.m[0:3, 0:3])
        return Rotation(self.m[0:3, 0:3])
    @property
    def rotation(self):
        return Rotation(self.m[0:3, 0:3])
    @rotation.setter
    def rotation(self, val):
        if isinstance(val, Rotation):
            self.m[0:3, 0:3] = val.m
        elif isinstance(val, np.ndarray):
            assert(len(val.shape) == 2)
            assert(val.shape[0] == 3)
            assert(val.shape[1] == 3)
            self.m[0:3, 0:3] = val
        else:
            raise Exception('dont know how to set rotation')
    @property
    def translation(self):
        return Translation(self.m[0:3, 3])
    @translation.setter
    def translation(self, val):
        if isinstance(val, Translation):
            self.m[0:3, 3] = np.squeeze(val.m)
        elif isinstance(val, np.ndarray):
            #assert(len(val.shape) == 2)
            #assert(val.shape[0] == 3)
            #assert(val.shape[1] == 3)
            assert(val.size == 3)
            self.m[0:3, 3] = val
        else:
            raise Exception('dont know how to set translation')
    @property
    def matrix(self):
        return self.m
    @property
    def inv(self):
        return Transformation(np.linalg.inv(self.m))
    def convert_m_to_mm(self):
        # Converts in place. Assumes translation is currently in m, and makes it in mm.
        self.m[0:3, 3] = self.m[0:3, 3] * 1000.
    def convert_mm_to_m(self):
        # Converts in place. Assumes translation is currently in mm, and makes it in m.
        self.m[0:3, 3] = self.m[0:3, 3] / 1000.
    def scaled(self, factor):
        return Transformation(self.translation.scaled(factor), self.rotation)
    def scaled_m_to_mm(self):
        # Assumes translation is currently in m, and makes it in mm.
        return Transformation(self.translation.scaled_m_to_mm(), self.rotation)
    def scaled_mm_to_m(self):
        # Assumes translation is currently in mm, and makes it in m.
        return Transformation(self.translation.scaled_mm_to_m(), self.rotation)
    def transform_hpoint(self, p):
        # Premultiplies the given homogenous point (HPoint) by this transform.
        # Returns the transformed homogenous point.
        assert(isinstance(p, HPoint))
        return HPoint(np.matmul(self.m, p.m))
    def as_geometry_pose(self):
        return geometry_msgs.msg.Pose(position=self.translation.as_geometry_point(),
                                      orientation=self.rotation.as_geometry_orientation())
    @staticmethod
    def from_xyz_mm_ABC_degrees(xyzABC):
        transl = Translation(xyzABC[0:3])
        rot = Rotation.from_ABC_degrees(xyzABC[3:6])
        return Transformation(transl, rot)
    @staticmethod
    def compose(A, B):
        return Transformation(np.matmul(A.matrix, B.matrix))

class HPoint():
    # Homogenous representation of a point
    def __init__(self, *args):
        if len(args) == 3:
            self.m = np.reshape(np.array([args[0], args[1], args[2], 1.]), (4,1))
        elif len(args) == 1:
            if isinstance(args[0], np.ndarray) and args[0].size == 3:
                self.m = np.reshape(np.array([args[0][0], args[0][1], args[0][2], 1.]), (4,1))
            elif isinstance(args[0], np.ndarray) and args[0].size == 4:
                self.m = np.reshape(args[0], (4,1))
            else:
                raise Exception('Dont know how to init')
        else:
            raise Exception('Dont know how to init')
    def __str__(self):
        return str(self.m)
    @property
    def x(self):
        return self.m[0,0]
    @x.setter
    def x(self, val):
        self.m[0,0] = val
    @property
    def y(self):
        return self.m[1,0]
    @y.setter
    def y(self, val):
        self.m[1,0] = val
    @property
    def z(self):
        return self.m[2,0]
    @z.setter
    def z(self, val):
        self.m[2,0] = val
    @property
    def xyz(self):
        return self.m[0:3,0]

class Robot(object):
    def __init__(self):
        pass
    def __str__(self):
        return self.name
    @staticmethod
    def create_iiwa():
        r = Robot()
        r.name = 'Kuka iiwa R14'
        # End effector transofrmation matrix in straight-up home position (mm)
        r.home = np.array([[1., 0., 0., 0.],
                           [0., 1., 0., 0.],
                           [0., 0., 1., 1306.],
                           [0., 0., 0., 1.]])
        # Directions of axes in straight-up home position
        r.axes = np.array([[0, 0, 1],
                           [0, 1, 0],
                           [0, 0, 1],
                           [0, -1, 0],
                           [0, 0, 1],
                           [0, 1, 0],
                           [0, 0, 1]])
        # Location of each joint center, in straight-up home position (mm)
        r.offset = np.array([[0, 0, 170],
                             [0, 0, 360],
                             [0, 0, 600],
                             [0, 0, 780],
                             [0, 0, 1000],
                             [0, -60, 1180],
                             [0, 0, 1271]])
        # DOF from arrays we defined already
        r.dof = r.axes.shape[0]
        # Calculate screw vectors
        # Each axis is a column, so the result is 6 x dof.
        r.screw = np.empty([6, r.dof])
        for i in range(r.dof):
            r.screw[0:3, i] = r.axes[i, :].transpose()
            r.screw[3:6, i] = -np.cross(r.axes[i, :].transpose(), r.offset[i, :].transpose())
        # Return the resulting struct
        return r
    def FK_space(self, joint_angles):
        """Returns transformation matrix for end effector of the given robot at the given
        joint angles.

        joint_angles: list/vector of joints in radians

        Returns 4x4 transformation matrix of end effector"""
        cum_t = np.eye(4)
        for i in range(self.dof):
            skew_s = skewsym(self.screw[:, i])
            cum_t = np.matmul(cum_t, linalg.expm(skew_s * joint_angles[i]))
        return Transformation(np.matmul(cum_t, self.home))





def _unit_test_FK_space():
    r = Robot.create_iiwa()
    # Matlab example 4, ans x/y/z/A/B/C = -636, -158, 1012, 68, -43, -35
    ja_deg = [-0.01, -35.10, 47.58, 24.17, 0.00, 0.00, 0.00]
    ja_rad = np.deg2rad(ja_deg)
    t = r.FK_space(ja_rad)
    assert(np.allclose(t.rotation.as_zyx(degrees=True), [68.28584782, -43.53993415, -35.84364870], atol=1e-8))
    assert(np.allclose(t.rotation.as_rpy(degrees=True), [-35.844, -43.540, 68.286], atol=1e-8))
    assert(np.allclose(t.translation.m, [-636.32792290, -158.87809407, 1012.70702237], atol=1e-8))

def _unit_test_Rotation_from_ABC_vs_legacy():
    '''Compare to a legacy standalone function. Which prob doesn't exist any longer.'''
    def ABCdeg_to_quat(adeg, bdeg, cdeg):
        '''Legacy function, from https://doc.rc-visard.com/v21.07/en/pose_format_kuka.html'''
        (Ar, Br, Cr) = (deg2rad(d) for d in (adeg, bdeg, cdeg))
        x = cos(Ar/2) * cos(Br/2) * sin(Cr/2) - sin(Ar/2) * sin(Br/2) * cos(Cr/2) 
        y = cos(Ar/2) * sin(Br/2) * cos(Cr/2) + sin(Ar/2) * cos(Br/2) * sin(Cr/2)
        z = sin(Ar/2) * cos(Br/2) * cos(Cr/2) - cos(Ar/2) * sin(Br/2) * sin(Cr/2) 
        w = cos(Ar/2) * cos(Br/2) * cos(Cr/2) + sin(Ar/2) * sin(Br/2) * sin(Cr/2)
        return (x, y, z, w)
    q = Rotation.from_ABC_degrees([20,30,-40]).as_quat()
    qxyzw = (q.x, q.y, q.z, q.w)
    q2 = ABCdeg_to_quat(20,30,-40)
    assert(np.allclose(qxyzw, q2, atol=1e-12))
    q = Rotation.from_ABC_degrees([-15,22,10]).as_quat()
    qxyzw = (q.x, q.y, q.z, q.w)
    q2 = ABCdeg_to_quat(-15,22,10)
    assert(np.allclose(qxyzw, q2, atol=1e-12))
    q = Rotation.from_ABC_degrees([0,190,-600]).as_quat()
    qxyzw = (q.x, q.y, q.z, q.w)
    q2 = ABCdeg_to_quat(0,190,-600)
    assert(np.allclose(qxyzw, q2, atol=1e-12))

def _unit_test_Rotation_from_zyx():
    assert(np.allclose(Rotation.from_zyx([0, 0, 0]).m, np.array([[1,0,0],[0,1,0],[0,0,1]])))
    assert(np.allclose(quaternion.as_float_array(Rotation.from_zyx([0, 0, 0]).as_quat()), np.array([1, 0, 0, 0]), atol=1e-12))

    assert(quaternion.isclose(Rotation.from_zyx([pi * 1.001, 0, pi * 1.001]).as_quat(), np.quaternion(0.000002467399071, -0.001570793742940, 0.999997532600929, -0.001570793742940)))
    assert(quaternion.isclose(Rotation.from_zyx([pi * 0.999, 0, pi * 0.999]).as_quat(), np.quaternion(0.000002467399071, 0.001570793742940, 0.999997532600929, 0.001570793742940)))
    assert(quaternion.isclose(Rotation.from_zyx([pi, 0, pi]).as_quat(), np.quaternion(0.0, 0.0, 1.0, 0.0)))
    print(Rotation.from_zyx([pi/4, 0, pi]).as_quat(), np.quaternion(0.0, 0.923879532511287, 0.382683432365090, 0.000000005268356))
    print(Rotation.from_ABC([pi/4, 0, pi]).as_quat(), np.quaternion(0.0, 0.923879532511287, 0.382683432365090, 0.000000005268356))
    assert(quaternion.isclose(Rotation.from_zyx([pi/4, 0, pi]).as_quat(), np.quaternion(0.0, 0.923879532511287, 0.382683432365090, 0.000000005268356), atol=1e-3))

def _unit_test_Rotation_zyx_is_ABC(verbose=False):
    egs = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [pi/2, -pi/4, 0]]
    if verbose: print('from_ABC -> as_ABC')
    for eg in egs:
        ans = Rotation.from_ABC(eg).as_ABC()
        if verbose: print(eg, ans, ans - eg)
        assert(np.allclose(ans - eg, 0, atol=1e-10))
    if verbose: print('from_ABC -> as_zyx')
    for eg in egs:
        ans = Rotation.from_ABC(eg).as_zyx()
        if verbose: print(eg, ans, ans - eg)
        assert(np.allclose(ans - eg, 0, atol=1e-10))
    if verbose: print('from_zyx -> as_zyx')
    for eg in egs:
        ans = Rotation.from_zyx(eg).as_zyx()
        if verbose: print(eg, ans, ans - eg)
        assert(np.allclose(ans - eg, 0, atol=1e-10))
    if verbose: print('from_zyx -> as_ABC')
    for eg in egs:
        ans = Rotation.from_zyx(eg).as_ABC()
        if verbose: print(eg, ans, ans - eg)
        assert(np.allclose(ans - eg, 0, atol=1e-10))
    if verbose: print('Rotation matrices from_ABC followed by from zyx')
    for eg in egs:
        anszyx = Rotation.from_zyx(eg)
        ansabc = Rotation.from_ABC(eg)
        if verbose: print(ansabc), print(anszyx)
        assert(np.allclose(anszyx.m, ansabc.m, atol=1e-10))

def _unit_test_Rotation_rpy_is_zyx_reversed(verbose=False):
    egs = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [pi/2, -pi/4, 0]]
    for eg in egs:
        anszyx = Rotation.from_zyx(eg)
        ansrpy = Rotation.from_rpy(np.flip(eg))
        if verbose: print(ansabc), print(ansrpy)
        assert(np.allclose(anszyx.m, ansrpy.m, atol=1e-10))
    for eg in egs:
        rpy = Rotation.from_zyx(eg).as_rpy()
        assert(np.allclose(np.flip(rpy), eg, atol=1e-10))

def _unit_test():
    _unit_test_FK_space()
    _unit_test_Rotation_from_ABC_vs_legacy()
    #_unit_test_Rotation_from_zyx()
    _unit_test_Rotation_zyx_is_ABC()
    _unit_test_Rotation_rpy_is_zyx_reversed()

if __name__ == '__main__':
    _unit_test()