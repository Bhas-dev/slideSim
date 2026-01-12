import numpy as np
from src.abstractObject import AbstractObject

class StaticObject(AbstractObject):

    def __init__(self, calc):
        self.adhesion_coeff = 0.000000000000001
        super().__init__(0, np.array([0,0,0]), calc, True)