from abc import ABC, abstractmethod
import numpy as np
class AbstractObject(ABC):
    def __init__(self, mass, center, calc, static=False):
        self.center = center
        self.mass = mass
        self.static = static
        self.calculator = calc
        self.calcIdx = self.calculator.addObject(self)

    def calculateHitbox(self):
        self.hitbox.clear()
        for i in range(self.nbVertices):
            self.hitbox.append([self.vertices_gnd[i], self.vertices_gnd[(i+1)%self.nbVertices]])

    def getPositions(self):
        return self.vertices_gnd.flatten()

