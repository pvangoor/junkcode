import LieGroup
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
        return NotImplemented
    
    def __truediv__(self, other):
        if isinstance(other, SO3):
            result = SO3()
            result._rot = self._rot * other._rot.inv()
            return result
        elif isinstance(other, np.ndarray) and other.shape == (3,1):
            return self._rot.as_matrix() @ other
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
    def readFromCSV(line, format_spec="q"):
        # Possible formats are
        # R : 9 entry matrix (row-by-row)
        # q : 4 entry quaternion (scalar last)
        # w : 4 entry quaternion (scalar first)
        # r : 3 entry log vector
        result = SO3()
        if format_spec == "R":
            mat = np.reshape(np.array([float(line[i]) for i in range(9)]), (3,3))
            result._rot.from_matrix(mat)
        elif format_spec == "q":
            quat = np.array([float(line[i]) for i in range(4)])
            result._rot.from_quat(quat)
        elif format_spec == "w":
            quat = np.array([float(line[i]) for i in [1,2,3,0]])
            result._rot.from_quat(quat)
        elif format_spec == "r":
            rotvec = np.array([float(line[i]) for i in range(3)])
            result._rot.from_rotvec(rotvec)
        else:
            return NotImplemented
        return result

    def writeToCSV(self, format_spec):
        pass

if __name__ == "__main__":
    R = SO3()