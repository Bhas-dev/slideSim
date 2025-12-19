from src.abstractObject import AbstractObject
import numpy as np

class DynamicObject(AbstractObject):

    def __init__(self, mass, center, static=False):
        super().__init__(mass, center, static)
        self.velocity = np.array([0.0, 0.0])

    def updateObject(self):
        pass

