import numpy as np
from src.abstractObject import AbstractObject

class StaticObject(AbstractObject):

    def __init__(self, hitbox, calc):
        super().__init__(0, np.array([0,0,0]), calc, True)
        self.hitbox = hitbox
    
    def initHitbox(self):
        self.hitbox = []
        for i in range(len(self.vertices)//2-2):
            self.hitbox.append(self.vertices[2*i:2*(i+2)])
    
    def getPositions(self):
        return self.vertices.flatten()