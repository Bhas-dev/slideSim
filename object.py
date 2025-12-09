import numpy as np

class Object():
    def __init__(self, filename = None):
        """takes a file as input, finds a way to draw 2d object from it, square by default"""
        if filename is None: # square with uniform weight distribution
            L = 5 #cm
            self.mass = 3 #kg
            self.center = np.array([0,0,0]) # center position in unmoving frame
            self.attitude = np.array([[1,0,0], [0,1,0], [0,0,1]]) #body to unmoving frame, attitude = attitude @ rotation
            A = np.array([[-L/2],[-L/2],[0]])
            B = np.array([[-L/2],[L/2],[0]])
            C = np.array([[L/2],[L/2],[0]])
            D = np.array([[L/2],[-L/2],[0]])
            L1 = (A, B)
            L2 = (B, C)
            L3 = (C, D)
            L4 = (D, A) #this one might be redundant, since it's just the last point linked to the first one
            self.hitbox = [L1, L2, L3, L4] # list of segments in body frame, in the future, a function should calculate this bounding box
    
    def updateObject(self):
        cacaaaaa