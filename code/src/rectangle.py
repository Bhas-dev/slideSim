import numpy as np

from calculator import Calculator
from src.dynamicObject import DynamicObject

class Rectangle(DynamicObject):
    def __init__(self, calc, h = 0.1, w = 0.25, mass = 50, center = np.array([0,0]), attitude = np.array([[1,0], [0,1]]), adhesion = 0.5):

        super().__init__(mass, center, calc)
        """takes a file as input, finds a way to draw 2d object from it, square by default"""
        self.inertia = (1/12) * self.mass * (w**2+h**2)
        self.size = np.array([w, h])
        self.adhesion_coeff = adhesion
        self.attitude = attitude # body to unmoving frame, new_attitude = attitude @ rotation
        self.vertices_bdy = np.array([[-w/2, -h/2],   # x0, y0
                                    [-w/2, h/2],   # x1, y1
                                    [w/2, h/2],   # x2, y2
                                    [w/2, -h/2]],  # x3, y3
                                np.float64)
        self.vertices_gnd = (self.attitude @ self.vertices_bdy.T).T
        self.nbVertices = len(self.vertices_gnd)
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


