from src.abstractObject import AbstractObject
import numpy as np

class DynamicObject(AbstractObject):

    def __init__(self, mass, center, calc, velocity):
        super().__init__(mass, center, calc)
        self.velocity = velocity

    def updateObject(self):
        net_force = self.calculator.calculateForces(self) 
        self.center, self.velocity = self.calculator.integrate_motion(
            mass=self.mass,
            current_position=self.center,
            current_velocity=self.velocity,
            net_force=net_force,
            dt=0.01
        )

        # self.attitude = self.calculator.rotate(self.attitude, np.pi/6)

        new_vertices_gnd = (self.attitude @ self.vertices_bdy.T).T

        for r in range(len(new_vertices_gnd)):
            new_vertices_gnd[r] += self.center

        diff = new_vertices_gnd - self.vertices_gnd
        self.vertices_gnd = new_vertices_gnd
        self.calculateHitbox()
        # diff = np.array([[0,0],[0,0],[0,0],[0,0]])
        return diff
    
    def calculateHitbox(self):
        self.hitbox.clear()
        for i in range(len(self.vertices_gnd)//2-2):
            self.hitbox.append(self.vertices_gnd[2*i:2*(i+2)])

