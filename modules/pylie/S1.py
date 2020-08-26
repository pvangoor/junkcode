import LieGroup
import numpy as np

class S1(LieGroup.LieGroup):
    # The Lie group of dim 1 scaling.
    # [ s ]
    def __init__(self, s = 1.0):
        self._scale = 1.0
    
    def Adjoint(self):
        return np.eye(1)
    
    def __mul__(self, other):
        if isinstance(other, S1):
            result = S1()
            result._scale = self._scale * other._scale
            return result
        return NotImplemented
    
    def __truediv__(self, other):
        if isinstance(other, S1):
            result = S1()
            result._scale = self._scale / other._scale
            return result
        elif isinstance(other, np.ndarray):
            return self._scale * other
        return NotImplemented
    
    def inv(self):
        result = S1()
        result._scale = 1.0/self._scale
        return result
    
    def log(self):
        return np.log(self._scale)
    
    def as_matrix(self):
        return np.array(self._scale)
    
    @staticmethod
    def identity():
        result = S1()
        result._scale = 1.0
        return result

    @staticmethod
    def exp(tr3vec):
        if isinstance(tr3vec, np.ndarray):
            assert tr3vec.shape == (1,1), "Invalid shape of Lie algebra vector."
            tr3vec = float(tr3vec)
        result = S1()
        result._scale = np.exp(tr3vec)
        return result

    @staticmethod
    def validFormats() -> list:
        # Possible formats are
        # s : 1 entry scale
        return ['s']

    @staticmethod
    def read_from_csv(line, format_spec="s") -> 'S1':
        result = S1()
        if format_spec == "s":
            result._scale = float(line[0])
        else:
            return NotImplemented
        return result

    def write_to_csv(self, format_spec) -> list:
        if format_spec == "s":
            result = [float(self._scale)]
        else:
            return NotImplemented
        return result
    
    @staticmethod
    def gen_csv_header(format_spec):
        if format_spec == "s":
            result = ["s"]
        else:
            return NotImplemented
        return result

if __name__ == "__main__":
    s = S1()