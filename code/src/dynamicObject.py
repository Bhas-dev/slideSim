from src.abstractObject import AbstractObject
import numpy as np

class DynamicObject(AbstractObject):

    def __init__(self, mass, center, calc, velocity = np.array([.0, .0])):
        super().__init__(mass, center, calc)
        self.velocity = velocity
        self.angular_velocity = 0.0
        self.attitude = np.array([[1,0],[0,1]])
        self.angle = 0.0
        

    def updateObject(self):
        net_force = self.calculator.calculateForces(self)[0]
        net_torque = self.calculator.calculateForces(self)[1]
        self.center, self.velocity, self.angle, self.angular_velocity = self.calculator.integrate_motion(
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
        self.attitude = self.calculator.rotate(self.attitude, self.angular_velocity * 0.01)

        # self.attitude = self.calculator.rotate(self.attitude, np.pi/6)

        new_vertices_gnd = (self.attitude @ self.vertices_bdy.T).T

        for r in range(len(new_vertices_gnd)):
            new_vertices_gnd[r] += self.center

        diff = new_vertices_gnd - self.vertices_gnd
        self.vertices_gnd = new_vertices_gnd
        self.calculateHitbox()
        # diff = np.array([[0,0],[0,0],[0,0],[0,0]])
        return diff

