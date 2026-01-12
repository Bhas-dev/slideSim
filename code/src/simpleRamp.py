import numpy as np
from src.staticObject import StaticObject

class SimpleRamp(StaticObject):
    def __init__(self, calc):

        self.vertices_gnd = np.array([[-5., -2],
                          [-5.,1],
                          [-3.5,-0.25],
                          [-2., -0.5],
                          [-1.5, .0],
                          [-1.5, -2],
                          [-2., -2],
                          [-3.5, -2]])
        
        self.vertices_gnd += np.array([2.25,0])
        
        # Define the colours for each vertex

        self.colours = np.array([.200, .200, .200,  # (r, g, b) to make gray
                        .200, .200, .200,
                        .200, .200, .200,
                        .200, .200, .200,
                        .200, .200, .200,
                        .200, .200, .200,
                        .200, .200, .200,
                        .200, .200, .200])

         # Define the triangles for the ramp mesh

        self.indices = np.array([0, 1, 2,
                        0, 2, 7,
                        7, 2, 3,
                        7, 3, 6,
                        6, 3, 4,
                        6, 4, 5])
        

        self.nbVertices = len(self.vertices_gnd)
        self.hitbox = []
        self.setHitbox()
        super().__init__(calc)
    
    def setHitbox(self):
        self.hitbox = [[[0, 1], [1, 2], [2, 7], [7, 0]], [[7, 2], [2, 3], [3, 6], [6, 7]], [[6, 3], [3, 4], [4, 5], [5, 6]]]