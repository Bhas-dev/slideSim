import numpy as np
from src.staticObject import StaticObject

class Floor(StaticObject):
    def __init__(self, calc):
        self.vertices = np.array([-5., -4.,
                          -5.,-1.8,
                          5.,-1.8,
                          5., -4.],
                         np.float64)

        self.colours = np.array([.333, .333, .334,  # (r, g, b) to make gray
                        .333, .333, .334,
                        .333, .333, .334,
                        .333, .333, .334])

        self.indices = np.array([0, 1, 2,
                        0, 2, 3])

        hitbox = self.initHitbox()
        super().__init__(hitbox, calc)

