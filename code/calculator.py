import numpy as np

G = 9.81 #m.s-1


class Calculator:
    def move(self):
        return np.array([0.01, 0])
    
    def Gravity(self, obj):
        return (obj.mass * G) * np.array([0, -1]) #m*g*vect(-y)
    
    def friction_coef(self, obj1, obj2):
        return (0.05 + 0.8 * min(obj1.adhesion_coeff, obj2.adhesion_coeff))
    
    def friction_force(self, obj, friction_coef):
        angle = np.arctan2(obj.attitude[1][0], obj.attitude[0][0])
        return (- friction_coef * obj.mass * G * np.cos(angle) * obj.velocity/np.linalg.norm(obj.velocity))


    def air_resistance(self, obj):
        drag_coef = 0.47 #spherical object
        air_density = 1.225 #kg/m3
        area = 1.0 #m2
        v = np.linalg.norm(obj.velocity)
        drag_magnitude = 0.5 * drag_coef * air_density * area * v**2
        if v != 0:
            drag_direction = -obj.velocity / v
        else:
            drag_direction = np.array([0.0, 0.0])
        return drag_magnitude * drag_direction
    

    



    def rotate(self, orientation, angle):
        c, s = np.cos(angle), np.sin(angle)
        R = np.array([[c, -s], [s, c]])
        return R @ orientation
    
    def calculateForces(self, obj):
        weight = np.array([0, -9.81*obj.mass]) # N
        resulting_force = weight
        return resulting_force

    def convertToMovement(self, ):
        pass