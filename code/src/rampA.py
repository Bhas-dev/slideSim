import numpy as np
from src.staticObject import StaticObject

class RampA(StaticObject):
    def __init__(self, calc):

        self.vertices_gnd = np.array([[-5., -2],
                          [-5.,1],
                          [-1.5, -2]])
        
        self.vertices_gnd += np.array([2.25,0])
        
        # Define the colours for each vertex

        self.colours = np.array([.200, .200, .200,  # (r, g, b) to make gray
                        .200, .200, .200,
                        .200, .200, .200])

         # Define the triangles for the ramp mesh

        self.indices = np.array([0, 1, 2])
        

        self.nbVertices = len(self.vertices_gnd)
        self.hitbox = []
        self.setHitbox()
        super().__init__(calc)