import numpy as np

class Object():
    def __init__(self, filename = None):
        """takes a file as input, finds a way to draw 2d object from it, square by default"""
        if filename is None: # square with uniform weight distribution
            
            self.initSquare(center = np.array([2.,3.]))

    def updateObject(self):
        self.vertices_gnd = self.attitude @ self.vertices_bdy.T + self.center.reshape(2,1)
        self.calculateHitbox()
        
    def initSquare(self, center, attitude = np.array([[1,0], [0,1]]) ):
        """takes a file as input, finds a way to draw 2d object from it, square by default"""
        
        self.mass = 3 #kg
        self.center = center # center position in unmoving frame
        self.attitude = attitude #body to unmoving frame, attitude = attitude @ rotation
        self.vertices_gnd 
        self.vertices_bdy = np.array([[0., 0.],   # x0, y0
                                    [1., 0.],   # x1, y1
                                    [0., 1.],   # x2, y2
                                    [1., 1.]],  # x3, y3
                                np.float64)
        colours = np.array([1., 0., 0.,  # (r, g, b) for vertex 0
                            0., 0., 1.,  # (r, g, b) for vertex 1
                            0., 1., 0.,  # ...
                            1., 1., 1.]) # ...
        indices = np.array([0, 1, 2,   # First triangle composed by vertices 0, 1 and 2
                            1, 2, 3])  # Second triangle composed by vertices 1, 2 and 3
        self.hitbox = [] # list of segments in body frame, in the future, a function should calculate this bounding box
    

    def caculateHitbox(self):
        for i in range(len(self.vertices_gnd)//2-2):
            self.hitbox.append(self.vertices_gnd[2*i:2*(i+2)])


