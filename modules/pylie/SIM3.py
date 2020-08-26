import LieGroup
from SE3 import SE3 as SE3
from SO3 import SO3 as SO3
from R3 import R3 as R3
from S1 import S1 as S1
import numpy as np

class SIM3(LieGroup.LieGroup):
    # SIM(3) is defined with the matrix form
    # [ sR x ]
    # [ 0  1 ]
    def __init__(self, R = SO3(), x = R3, s = S1):
        self._R = R
        self._x = x
        self._s = s
    
    def Adjoint(self):
        return NotImplemented
    
    def __mul__(self, other):
        if isinstance(other, SIM3):
            result = SIM3()
            result._R = self._R * other._R
            result._x = self._x + self._s * (self._R * other._x)
            result._s = self._s * other._s
            return result
        if isinstance(other, np.ndarray):
            if other.shape == (3,1):
                return self._x + self._s * (self._R * other)
            elif other.shape == (4,1):
                return self.as_matrix() @ other
        
        return NotImplemented
    
    def as_matrix(self):
        mat = np.eye(4)
        mat[0:3,0:3] = self._s * self._R.as_matrix()
        mat[0:3,3:4] = self._x
        return mat
    
    def __truediv__(self, other):
        if isinstance(other, SIM3):
            return self * other.inv()
        return NotImplemented
    
    def inv(self):
        result = SIM3()
        result._R = self._R.inv()
        result._x = - self._s.inv() * ( self._R.inv() * self._x )
        result._s = self._s.inv()
        return result
    
    def log(self):
        return NotImplemented
    
    @staticmethod
    def exp(sim3vec):
        assert sim3vec.shape == (7,1), "Invalid shape of Lie algebra vector."
        return NotImplemented

    @staticmethod
    def validFormats():
        # Possible formats are
        # SO(3) format specs
        # R(3) format specs
        # S(1) format specs
        return SO3.validFormats() + R3.validFormats() + S1.validFormats() + ['S']

    @staticmethod
    def read_from_csv(line, format_spec="sqx") -> 'SE3':
        result = SE3()
        SO3_formats = SO3.validFormats()
        R3_formats = R3.validFormats()
        S1_formats = S1.validFormats()
        for fspec in format_spec:
            if fspec in SO3_formats:
                result._R = SO3.read_from_csv(line)
            elif fspec in R3_formats:
                result._x = R3.read_from_csv(line)
            elif fspec in S1_formats:
                result._s = S1.read_from_csv(line)
            else:
                return NotImplemented
        return result

    def write_to_csv(self, format_spec) -> list:
        result = []
        SO3_formats = SO3.validFormats()
        R3_formats = R3.validFormats()
        S1_formats = S1.validFormats()
        for fspec in format_spec:
            if fspec in SO3_formats:
                result += self._R.write_to_csv(fspec)
            elif fspec in R3_formats:
                result += self._x.write_to_csv(fspec)
            elif fspec in S1_formats:
                result += self._s.write_to_csv(fspec)
            else:
                return NotImplemented
        return result
    
    @staticmethod
    def gen_csv_header(format_spec) -> list:
        result = []
        SO3_formats = SO3.validFormats()
        R3_formats = R3.validFormats()
        S1_formats = S1.validFormats()
        for fspec in format_spec:
            if fspec in R3_formats:
                result += R3.gen_csv_header(fspec)
            elif fspec in SO3_formats:
                result += SO3.gen_csv_header(fspec)
            elif fspec in S1_formats:
                result += S1.gen_csv_header(fspec)
            else:
                return NotImplemented
        return result

if __name__ == "__main__":
    S = SIM3()