import numpy as np

G = 9.81 #m.s-1


class Calculator:
    def move(self):
        return np.array([0.01, 0])
    
    def Gravity(self, obj):
        return (obj.mass * G) * np.array([0, -1]) #m*g*vect(-y)
    
    def rotate(self, orientation, angle):
        c, s = np.cos(angle), np.sin(angle)
        R = np.array([[c, -s], [s, c]])
        return R @ orientation
    
    def calculate_forces(self, obj):
        weight = np.array([0, -9.81*obj.mass]) # N
        resulting_force = weight
        return resulting_force