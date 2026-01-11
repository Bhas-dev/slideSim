from abc import ABC, abstractmethod
import numpy as np
class AbstractObject(ABC):
    def __init__(self, mass, center, calc, static=False):
        self.center = center
        self.mass = mass
        self.static = static
        self.calculator = calc
        self.calcIdx = self.calculator.addObject(self)

    def cross(self,a,b):
        return a[0]*b[1] - a[1]*b[0]

    def isConvex(self):
        for i in range(self.nbVertices):
            prev_v = self.vertices_gnd[i - 1]
            curr_v = self.vertices_gnd[i]
            next_v = self.vertices_gnd[(i + 1) % self.nbVertices]
            e1 = curr_v - prev_v
            e2 = next_v - prev_v 
            if self.cross(e1, e2) > 0:
                return False
        return True

    def setHitbox(self):
        if self.isConvex():
            self.hitbox = [[[i, (i+1)%self.nbVertices] for i in range(self.nbVertices)]]
        else:
            for i in range(len(self.indices)//3):
                j = 3*i
                self.hitbox.append([[self.indices[j], self.indices[j+1]], [self.indices[j+1], self.indices[j+2]], [self.indices[j+2], self.indices[j]]])

    def getPositions(self):
        return self.vertices_gnd.flatten()

