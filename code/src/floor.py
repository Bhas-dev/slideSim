import numpy as np
from src.staticObject import StaticObject

class Floor(staticObject):
    def __init__(self):
        hitbox = self.floorHitbox()
        super().__init__(hitbox)

    def floorHitbox(self):
        return [np.array([[-5,-1.8], [5,-1.8]])]


