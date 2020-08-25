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

if __name__ == "__main__":
    R = SO3()