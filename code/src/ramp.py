import numpy as np
from src.staticObject import StaticObject

class Ramp(StaticObject):
    def __init__(self, calc):
        p_start = [-4, 1]
        p_ctrl = [-1, -2.5]
        p_end = [1, -1.3]

        t = np.linspace(0, 1, 30)
        print(t)
        # Formule de Bézier quadratique
        surface_points = [[-5,-2]]
        for val in t:
            point = [0.0, 0.0]
            point[0] = (1-val)**2 * p_start[0] + 2*(1-val)*val * p_ctrl[0] + val**2 * p_end[0]
            point[1] = (1-val)**2 * p_start[1] + 2*(1-val)*val * p_ctrl[1] + val**2 * p_end[1]
            surface_points.append(point)
        surface_points.append([1,-2])

        self.vertices_gnd = np.array(surface_points)

         # Define the colours for each vertex

        self.colours = np.array([.200, .200, .200,  # (r, g, b) to make gray
        ])
        self.colours = np.tile(self.colours, 32)

         # Define the triangles for the ramp mesh

        self.indices = np.array([0, 1, 2,
                                0, 2, 3,
                                0, 3, 4,
                                0, 4, 5,
                                0, 5, 6,
                                0, 6, 7,
                                0, 7, 8,
                                0, 8, 9,
                                0, 9, 10,
                                0, 10, 11,
                                0, 11, 12,
                                0, 12, 13,
                                0, 13, 14,
                                0, 14, 15,
                                0, 15, 31,
                                31, 15, 16,
                                31, 16, 17,
                                31, 17, 18,
                                31, 18, 19,
                                31, 19, 20,
                                31, 20, 21,
                                31, 21, 22,
                                31, 22, 23,
                                31, 23, 24,
                                31, 24, 25,
                                31, 25, 26,
                                31, 26, 27,
                                31, 27, 28,
                                31, 28, 29,
                                31, 29 , 30
                                ])
        self.nbVertices = len(self.vertices_gnd)
        self.hitbox = []
        self.calculateHitbox()
        super().__init__(calc)