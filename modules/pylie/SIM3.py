import LieGroup
from SE3 import SE3 as SE3
from SO3 import SO3 as SO3
import numpy as np

class SE3(LieGroup.LieGroup):
    # SIM(3) is defined with the matrix form
    # [ sR x ]
    # [ 0  1 ]
    def __init__(self, R = SO3(), t = np.zeros((3,1)), s = 1.0):
        self._rot = R
        self._trans = t
        self._scale = s
    
    def Adjoint(self):
        pass
    
    def __mul__(self, other):
        if isinstance(other, SIM3):
            result = SIM3()
            result._rot = self._rot * other._rot
            result._trans = self._trans + self._scale * (self._rot * other._trans)
            result._scale = self._scale * other._scale
            return result
        if isinstance(other, np.ndarray):
            if other.shape == (3,1):
                return self._trans + self._scale * (self._rot * other)
            elif other.shape == (4,1):
                return self.as_matrix() @ other
        
        return NotImplemented
    
    def as_matrix(self):
        mat = np.eye(4)
        mat[0:3,0:3] = self._scale * self._rot.as_matrix()
        mat[0:3,3:4] = self._trans
        return mat
    
    def __truediv__(self, other):
        if isinstance(other, SIM3):
            return self * other.inv()
        return NotImplemented
    
    def inv(self):
        result = SIM3()
        result._rot = self._rot.inv()
        result._trans = - (1.0/self.scale) * ( self.rot.inv() * self._trans )
        result._scale = (1.0/self.scale)
        return result
    
    def log(self):
        pass
    
    @staticmethod
    def exp(sim3vec):
        assert sim3vec.shape == (7,1), "Invalid shape of Lie algebra vector."
        pass

    @staticmethod
    def validFormats():
        # Possible formats are
        # q/w/R/r/x/P : SE(3) format specs
        # s : 1 entry scale
        return SO3.validFormats() + ['x','P']

    @staticmethod
    def read_from_csv(line, format_spec="sqx") -> 'SE3':
        return NotImplemented

    def write_to_csv(self, format_spec) -> list:
        return NotImplemented
    
    @staticmethod
    def gen_csv_header(format_spec) -> list:
        return NotImplemented

if __name__ == "__main__":
    P = SE3()