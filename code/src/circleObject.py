import numpy as np

from calculator import Calculator
from src.dynamicObject import DynamicObject

class CircleObject(DynamicObject):
    def __init__(self, mass = 3, center = np.array([0.5,0.5]) , static=False, adhesion = 0.5, radius=0.5):

        super().__init__(mass, center, static)
        """takes a file as input, finds a way to draw 2d object from it, square by default"""
        
        self.adhesion_coeff = adhesion
        self.velocity = np.array([.0,.0])
        self.radius = radius  
        self.inertia = 0.5 * self.mass * (self.radius **2)
        self.angular_velocity = 0.0
        self.angle = 0.0

        N_POINTS = 30 # number of points to approximate the circle
        
        
        angles = np.linspace(0, 2 * np.pi, N_POINTS, endpoint=False)
        x_coords = self.radius * np.cos(angles)
        y_coords = self.radius * np.sin(angles)
        
        # the first vertex is the center for the simulation
        self.vertices_bdy = np.vstack([
            [0.0, 0.0], # Centre du cercle (index 0)
            np.stack([x_coords, y_coords], axis=1)
        ])
        
        # self.indices : Defines the triangles for the circle mesh
        self.indices = []
        for i in range(1, N_POINTS + 1):
            next_i = i + 1 if i < N_POINTS else 1
            self.indices.extend([0, i, next_i])
        self.indices = np.array(self.indices, np.int32)
        
        # initialisation of the global vertices positions
        self.vertices_gnd = self._update_vertices_gnd()
        
       
        self.colours = np.array([0.2, 0.2, 0.8] * len(self.vertices_bdy)) 
        self.hitbox = []
        self.calculateHitbox()

    def _update_vertices_gnd(self):
        """Calculate the coordinates of the circle in the global frame based on its center."""
       
        new_vertices_gnd = self.vertices_bdy.copy()
        
        # Add the position of the center to each vertex
        for r in range(len(new_vertices_gnd)):
            new_vertices_gnd[r] += self.center
        
        return new_vertices_gnd


    def updateObject(self):
        net_force = Calculator().calculateForces(self)[0]
        net_torque = Calculator().calculateForces(self)[1]
        self.center, self.velocity, self.angle, self.angular_velocity = Calculator.integrate_motion(
            mass=self.mass,
            current_position=self.center,
            current_velocity=self.velocity,
            net_force=net_force,
            current_angular_vel=self.angular_velocity,
            current_angle=self.angle,
            net_torque=net_torque,
            inertia=self.inertia,
            dt=0.01
        )

        #attitude = Calculator().rotate(self.attitude, np.pi/6)

        new_vertices_gnd = self._update_vertices_gnd()

        

        diff = new_vertices_gnd - self.vertices_gnd
        self.vertices_gnd = new_vertices_gnd
        self.calculateHitbox()
        return diff
        
        
    

    def calculateHitbox(self):
        self.hitbox.clear()
        for i in range(len(self.vertices_gnd)//2-2):
            self.hitbox.append(self.vertices_gnd[2*i:2*(i+2)])
    
    def getPositions(self):
        return self.vertices_gnd.flatten()


