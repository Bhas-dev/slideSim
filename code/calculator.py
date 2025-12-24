import numpy as np

G = 9.81 #m.s-1


class Calculator:

    def __init__(self):
        self.objects = []
    
    def addObject(self, obj):
        self.objects.append(obj)
        return len(self.objects) - 1

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
        resulting_force = weight + self.air_resistance(obj) # adding all forces
        return resulting_force

    @staticmethod
    def integrate_motion(mass, current_position, current_velocity, net_force, dt):
        """
        Performs one step of Forward Euler integration (time-stepping).
        
        This method is the standard way to numerically go from a sum of forces 
        to an object's new position and velocity.

        Args:
            mass (float): The mass of the object.
            current_position (np.array): The object's current 2D position (center).
            current_velocity (np.array): The object's current 2D velocity.
            net_force (np.array): The sum of forces acting on the object (2D vector).
            dt (float): The time step duration (e.g., 1.0/60.0 for 60 FPS).

        Returns:
            tuple: (new_position, new_velocity)
        """
        # 1. Calculate Acceleration (a = F/m)
        if mass == 0: #static objects
            return current_position, current_velocity
        
        acceleration = net_force / mass
        
        # 2. Update Velocity (v_new = v_old + a*dt)
        new_velocity = current_velocity + acceleration * dt
        
        # 3. Update Position (p_new = p_old + v_new*dt)
        # Using the new velocity for position update is often called Semi-Implicit Euler
        new_position = current_position + new_velocity * dt
        
        return new_position, new_velocity