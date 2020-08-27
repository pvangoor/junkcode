import LieGroup
from SO3 import SO3 as SO3
from R3 import R3 as R3
import numpy as np

class SE3(LieGroup.LieGroup):
    def __init__(self, R = SO3(), x = R3):
        self._R = R
        self._x = x
    
    def __str__(self):
        return str(self.as_matrix())
    
    def Adjoint(self):
        return NotImplemented
    
    def __mul__(self, other):
        if isinstance(other, SE3):
            result = SE3()
            result._R = self._R * other._R
            result._x = self._x + (self._R * other._x)
            return result
        if isinstance(other, np.ndarray):
            if other.shape[0] == 3:
                return self._x + (self._R * other)
            elif other.shape[0] == 4:
                return self.as_matrix() @ other
        
        return NotImplemented
    
    def as_matrix(self):
        mat = np.eye(4)
        mat[0:3,0:3] = self._R.as_matrix()
        mat[0:3,3:4] = self._x._trans
        return mat
    
    def __truediv__(self, other):
        if isinstance(other, SE3):
            return self * other.inv()
        return NotImplemented
    
    def inv(self):
        result = SE3()
        result._R = self._R.inv()
        result._x = - (self._R.inv() * self._x)
        return result
    
    def log(self):
        return NotImplemented
    
    @staticmethod
    def exp(se3vec):
        assert se3vec.shape == (6,1), "Invalid shape of Lie algebra vector."
        return NotImplemented

    @staticmethod
    def valid_list_formats() -> dict:
        # Possible formats are
        # q/w/R/r : SO(3) format specs
        # x : 3 entry translation
        # P : 12 entry homogeneous matrix (row-by-row)
        result = {'P':12}
        result.update(SO3.valid_list_formats())
        result.update(R3.valid_list_formats())
        return result

    @staticmethod
    def from_list(line, format_spec="qx") -> 'SE3':
        result = SE3()
        SO3_formats = SO3.valid_list_formats()
        R3_formats = R3.valid_list_formats()
        for fspec in format_spec:
            if fspec in SO3_formats:
                result._R = SO3.from_list(line)
                line = line[SO3_formats[fspec]:]
            elif fspec in R3_formats:
                result._x = R3.from_list(line)
                line = line[R3_formats[fspec]:]
            elif fspec == "P":
                mat = np.reshape(np.array([float(line[i]) for i in range(12)]), (3,4))
                result._R._rot.from_matrix(mat[0:3,0:3])
                result._x._trans = mat[0:3,3:4]
                line = line[12:]
            else:
                return NotImplemented
        return result

    def to_list(self, format_spec) -> list:
        result = []
        SO3_formats = SO3.valid_list_formats()
        R3_formats = R3.valid_list_formats()
        for fspec in format_spec:
            if fspec in SO3_formats:
                result += self._R.to_list(fspec)
            elif fspec in R3_formats:
                result += self._x.to_list(fspec)
            elif fspec == "P":
                posemat = np.hstack((self._R.as_matrix(), self._x))
                result += posemat.ravel().tolist()
            else:
                return NotImplemented
        return result
    
    @staticmethod
    def list_header(format_spec) -> list:
        result = []
        SO3_formats = SO3.valid_list_formats()
        R3_formats = R3.valid_list_formats()
        for fspec in format_spec:
            if fspec == "P":
                result += "P11,P12,P13,P14,P21,P22,P23,P24,P31,P32,P33,P34".split()
            elif fspec in R3_formats:
                result += R3.list_header(fspec)
            elif fspec in SO3_formats:
                result += SO3.list_header(fspec)
            else:
                return NotImplemented
        return result

if __name__ == "__main__":
    P = SE3()