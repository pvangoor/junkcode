from abc import ABC, abstractmethod
import numpy as np

class LieGroup(ABC):
    @abstractmethod
    def __mul__(self, other):
        pass
    
    @abstractmethod
    def inv(self):
        pass
    
    @abstractmethod
    def __truediv__(self):
        pass
    
    @abstractmethod
    def log(self):
        pass
    
    @abstractmethod
    def Adjoint(self):
        pass

    @abstractmethod
    def as_matrix(self):
        pass
    
    @staticmethod
    def identity():
        pass

    @staticmethod
    def exp(lie_alg_vector):
        pass

    @staticmethod
    def valid_list_formats():
        return []
    
    @staticmethod
    def from_list(line, format_spec):
        pass
    
    @abstractmethod
    def to_list(self, format_spec):
        pass

    @staticmethod
    def list_header(format_spec):
        pass