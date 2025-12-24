import numpy as np

from calculator import Calculator
from src.dynamicObject import DynamicObject

class Square(DynamicObject):
    def __init__(self, calc, L = 0.25, mass = 3, center = np.array([0,0]), attitude = np.array([[1,0], [0,1]]), adhesion = 0.5):

        super().__init__(mass, center, calc)
        """takes a file as input, finds a way to draw 2d object from it, square by default"""
        self.size = L
        self.adhesion_coeff = adhesion
        self.attitude = attitude # body to unmoving frame, new_attitude = attitude @ rotation
        self.vertices_bdy = np.array([[-L/2, -L/2],   # x0, y0
                                    [-L/2, L/2],   # x1, y1
                                    [L/2, L/2],   # x2, y2
                                    [L/2, -L/2]],  # x3, y3
                                np.float64)
        self.vertices_gnd = (self.attitude @ self.vertices_bdy.T).T
        for r in range(len(self.vertices_gnd)):
            self.vertices_gnd[r] += self.center
        self.colours = np.array([1., 0., 0.,  # (r, g, b) for vertex 0
                            0., 0., 1.,  # (r, g, b) for vertex 1
                            0., 1., 0.,  # ...
                            1., 1., 1.]) # ...
        self.indices = np.array([0, 1, 2,   # First triangle composed by vertices 0, 1 and 2
                            0, 2, 3])  # Second triangle composed by vertices 1, 2 and 3
        self.hitbox = [] # list of segments in body frame, list of 2*2 numpy arrays
        self.calculateHitbox()


