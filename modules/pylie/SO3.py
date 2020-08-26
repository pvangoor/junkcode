import LieGroup
from R3 import R3 as R3
from scipy.spatial.transform import Rotation
import numpy as np

class SO3(LieGroup.LieGroup):
    def __init__(self, R = Rotation.identity()):
        self._rot = R
    
    def Adjoint(self):
        return self._rot.as_matrix()
    
    def __mul__(self, other):
        if isinstance(other, SO3):
            result = SO3()
            result._rot = self._rot * other._rot
            return result
        elif isinstance(other, np.ndarray) and other.shape == (3,1):
            return self._rot.as_matrix() @ other
        elif isinstance(other, R3):
            result = R3()
            result._trans = self._rot.as_matrix() @ other._trans
            return result
        return NotImplemented
    
    def __truediv__(self, other):
        if isinstance(other, SO3):
            result = SO3()
            result._rot = self._rot * other._rot.inv()
            return result
        return NotImplemented
    
    def inv(self):
        result = SO3()
        result._rot = self._rot.inv()
        return result
    
    def log(self):
        return self._rot.as_rotvec
    
    def as_matrix(self):
        return self._rot.as_matrix()
    
    @staticmethod
    def identity():
        result = SO3()
        result._rot = Rotation.identity()
        return result

    @staticmethod
    def exp(so3vec):
        assert so3vec.shape == (3,1), "Invalid shape of Lie algebra vector."
        result = SO3()
        result._rot.from_rotvec(so3vec)
        return result

    @staticmethod
    def validFormats() -> list:
        # Possible formats are
        # R : 9 entry matrix (row-by-row)
        # q : 4 entry quaternion (scalar last)
        # w : 4 entry quaternion (scalar first)
        # r : 3 entry log vector
        return ['R', 'q', 'w', 'r']

    @staticmethod
    def read_from_csv(line, format_spec="q") -> 'SO3':
        result = SO3()
        if format_spec == "R":
            mat = np.reshape(np.array([float(line[i]) for i in range(9)]), (3,3))
            result._rot = Rotation.from_matrix(mat)
            line = line[9:]
        elif format_spec == "q":
            quat = np.array([float(line[i]) for i in range(4)])
            result._rot = Rotation.from_quat(quat)
            line = line[4:]
        elif format_spec == "w":
            quat = np.array([float(line[i]) for i in [1,2,3,0]])
            result._rot = Rotation.from_quat(quat)
            line = line[4:]
        elif format_spec == "r":
            rotvec = np.array([float(line[i]) for i in range(3)])
            result._rot = Rotation.from_rotvec(rotvec)
            line = line[3:]
        else:
            return NotImplemented
        return result

    def write_to_csv(self, format_spec) -> list:
        # Possible formats are
        # R : 9 entry matrix (row-by-row)
        # q : 4 entry quaternion (scalar last)
        # w : 4 entry quaternion (scalar first)
        # r : 3 entry log vector
        if format_spec == "R":
            mat = self._rot.as_matrix()
            result = mat.ravel().tolist()
        elif format_spec == "q":
            quat = self._rot.as_quat()
            result = quat.ravel().tolist()
        elif format_spec == "w":
            quat = self._rot.as_quat()
            temp = quat.ravel().tolist()
            result = [temp[i] for i in [3,0,1,2]]
        elif format_spec == "r":
            rotvec = self._rot.as_rotvec()
            result = rotvec.ravel().tolist()
        else:
            return NotImplemented
        return result
    
    @staticmethod
    def gen_csv_header(format_spec):
        # Possible formats are
        # R : 9 entry matrix (row-by-row)
        # q : 4 entry quaternion (scalar last)
        # w : 4 entry quaternion (scalar first)
        # r : 3 entry log vector
        if format_spec == "R":
            result = "R11,R12,R13,R21,R22,R23,R31,R32,R33".split()
        elif format_spec == "q":
            result = "qx,qy,qz,qw".split()
        elif format_spec == "w":
            result = "qw,qx,qy,qz".split()
        elif format_spec == "r":
            result = "rx,ry,rz".split()
        else:
            return NotImplemented
        return result

if __name__ == "__main__":
    R = SO3()