import LieGroup
from SO3 import SO3 as SO3
import numpy as np

class SE3(LieGroup.LieGroup):
    def __init__(self, R = SO3(), t = np.zeros((3,1))):
        self._rot = R
        self._trans = t
    
    def Adjoint(self):
        pass
    
    def __mul__(self, other):
        if isinstance(other, SE3):
            result = SE3()
            result._rot = self._rot * other._rot
            result._trans = self._trans + self._rot * other._trans
            return result
        if isinstance(other, np.ndarray):
            if other.shape == (3,1):
                return self._rot * other + self._trans
            elif other.shape == (4,1):
                return self.as_matrix() @ other
        
        return NotImplemented
    
    def as_matrix(self):
        mat = np.eye(4)
        mat[0:3,0:3] = self._rot.as_matrix()
        mat[0:3,3:4] = self._trans
        return mat
    
    def __truediv__(self, other):
        if isinstance(other, SE3):
            return self * other.inv()
        return NotImplemented
    
    def inv(self):
        result = SE3()
        result._rot = self._rot.inv()
        result._trans = - self.rot.inv() * self._trans
        return result
    
    def log(self):
        pass
    
    @staticmethod
    def exp(so3vec):
        assert so3vec.shape == (6,1), "Invalid shape of Lie algebra vector."
        pass

if __name__ == "__main__":
    P = SE3()