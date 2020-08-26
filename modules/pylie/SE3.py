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
    def exp(se3vec):
        assert so3vec.shape == (6,1), "Invalid shape of Lie algebra vector."
        pass

    @staticmethod
    def validFormats():
        # Possible formats are
        # q/w/R/r : SO(3) format specs
        # x : 3 entry translation
        # P : 12 entry homogeneous matrix (row-by-row)
        return SO3.validFormats() + ['x','P']

    @staticmethod
    def read_from_csv(line, format_spec="qx") -> 'SE3':
        result = SE3()
        SO3_formats = SO3.validFormats()
        for fspec in format_spec:
            if fspec in SO3_formats:
                result._rot = SO3.read_from_csv(line)
            elif fspec == "x":
                pos = np.reshape(np.array([float(line[i]) for i in range(3)]), (3,1))
                result._trans = pos
                line = line[3:]
            elif fspec == "P":
                mat = np.reshape(np.array([float(line[i]) for i in range(12)]), (3,4))
                result._rot.from_quat(quat)
                line = line[12:]
            else:
                return NotImplemented
        return result

    def write_to_csv(self, format_spec) -> list:
        result = []
        SO3_formats = SO3.validFormats()
        for fspec in format_spec:
            if fspec in SO3_formats:
                result += self._rot.write_to_csv(fspec)
            elif fspec == "x":
                result += self._trans.ravel().tolist()
            elif fspec == "P":
                posemat = np.hstack((self._rot.as_matrix(), self._trans))
                result += posemat.ravel().tolist()
            else:
                return NotImplemented
        return result
    
    @staticmethod
    def gen_csv_header(format_spec) -> list:
        result = []
        SO3_formats = SO3.validFormats()
        for fspec in format_spec:
            if fspec == "P":
                result += "P11,P12,P13,P14,P21,P22,P23,P24,P31,P32,P33,P34".split()
            elif fspec == "x":
                result += "x1,x2,x3".split()
            elif fspec in SO3_formats:
                result += SO3.gen_csv_header(fspec)
            else:
                return NotImplemented
        return result

if __name__ == "__main__":
    P = SE3()