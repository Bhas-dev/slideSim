import numpy as np
import random
from src.staticObject import StaticObject

class Ramp(StaticObject):
    def __init__(self, calc):
        p_start = [-4, 1]
        p_ctrl = [-1, -2.5]
        p_end = [1, -1.3]

        t = np.linspace(0, 1, 10)
        print(t)
        # Formule de Bézier quadratique
        surface_points = [[-4.0, -2.0]]
        for val in t:
            point = [0.0, 0.0]
            point[0] = (1-val)**2 * p_start[0] + 2*(1-val)*val * p_ctrl[0] + val**2 * p_end[0]
            point[1] = (1-val)**2 * p_start[1] + 2*(1-val)*val * p_ctrl[1] + val**2 * p_end[1]
            surface_points.append(point)
            
        self.length = len(surface_points)-1
        for i in range(self.length-1):
            pt = [surface_points[self.length-i][0], -2.0]
            surface_points.append(pt)

        print(surface_points)
        print(self.length)

        self.vertices_gnd = np.array(surface_points)
        self.nbVertices = len(self.vertices_gnd)
        # Define the colours for each vertex

        self.colours = np.array([.200, .200, .200,  # (r, g, b) to make gray
        ])
        self.colours = np.tile(self.colours, self.length*2)
        

         # Define the triangles for the ramp mesh
        triangles = [0,1,2, 0,2,self.length*2 -1]
        for i in range(1,(self.length-1)):
            triangles+=[self.length*2-i, i+1, i+2, self.length*2 - i, i+2, self.length*2 - i -1]

        self.indices = np.array(triangles)
        
        self.hitbox = []
        self.setHitbox()
        super().__init__(calc)

    def setHitbox(self):
        self.hitbox = [[[0, 1],[1,2], [2, self.length*2 -1], [self.length*2 -1,0]]]
        for i in range (1,self.length-1):
            self.hitbox.append([[self.length*2 - i, i+1], [i+1, i+2], [i+2, self.length*2 - i -1], [self.length*2 - i -1, self.length*2 - i]])

        print(self.hitbox)