import LieGroup
import numpy as np

class R3(LieGroup.LieGroup):
    # The Lie group of dim 3 translation.
    # [ I_3  x ]
    # [  0   1 ]
    def __init__(self, x = np.zeros((3,1))):
        self._trans = x
    
    def Adjoint(self):
        return np.eye(3)
    
    def __mul__(self, other):
        if isinstance(other, R3):
            result = R3()
            result._trans = self._trans + other._trans
            return result
        return NotImplemented
    
    def __add__(self, other):
        return self * other
    
    def __truediv__(self, other):
        if isinstance(other, R3):
            result = R3()
            result._trans = self._trans - other._trans
            return result
        elif isinstance(other, np.ndarray):
            if other.shape == (3,1):
                return self._trans + other
            if other.shape == (4,1):
                return self.as_matrix() @ other
        return NotImplemented
    
    def __sub__(self, other):
        return self / other

    def inv(self):
        result = R3()
        result._trans = -self._trans
        return result
    
    def log(self):
        return self._trans
    
    def as_matrix(self):
        result = np.eye(4)
        result[0:3,3:4] = self._trans
        return result
    
    @staticmethod
    def identity():
        result = R3()
        result._trans = np.zeros((3,1))
        return result

    @staticmethod
    def exp(tr3vec):
        assert tr3vec.shape == (3,1), "Invalid shape of Lie algebra vector."
        result = R3()
        result._trans = tr3vec
        return result

    @staticmethod
    def validFormats() -> list:
        # Possible formats are
        # x : 3 entry vector
        return ['x']

    @staticmethod
    def read_from_csv(line, format_spec="x") -> 'R3':
        result = R3()
        if format_spec == "x":
            result._trans = np.reshape(np.array([float(line[i]) for i in range(3)]), (3,1))
            line = line[3:]
        else:
            return NotImplemented
        return result

    def write_to_csv(self, format_spec) -> list:
        if format_spec == "x":
            result = self._trans.ravel().tolist()
        else:
            return NotImplemented
        return result
    
    @staticmethod
    def gen_csv_header(format_spec):
        if format_spec == "x":
            result = "x1,x2,x3".split()
        else:
            return NotImplemented
        return result

if __name__ == "__main__":
    x = R3()