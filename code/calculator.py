import numpy as np
from src import Calculator


class Calculator:
    def move(self):

        return np.array([0.01, 0])
    

    
    def rotate(self,orientation, angle):
        c, s = np.cos(angle), np.sin(angle)
        R = np.array([[c, -s], [s, c]])
        return R @ orientation
    
    def calculate_forces(self, obj):
        weight = np.array([0, -9.81*obj.mass]) # N


        resulting_force = weight

        return resulting_force