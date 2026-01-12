from src.abstractObject import AbstractObject
import numpy as np
from src.floor import Floor
import glfw

class DynamicObject(AbstractObject):

    def __init__(self, mass, center, calc, velocity = np.array([.0, .0])):
        super().__init__(mass, center, calc)
        self.velocity = velocity
        self.angular_velocity = 0.0
        self.attitude = np.array([[1,0],[0,1]])
        self.angle = 0.0
    
    def resolveIntersections(self):
        for other in self.calculator.objects:
            if other is self:
                continue

            if isinstance(other, Floor):
                hit, info = self.calculator.intersectFloor(self, other)
            else:
                hit, info = self.calculator.intersect(self, other)
            if not hit:
                continue

            depth, normal, contact_pts = info

            # 1) velocity-level resolution (impulses)
            self.calculator.resolveCollision(self, other, normal, contact_pts)

            # 2) position-level correction
            self.calculator.positionalCorrection(self, normal, depth)

    def updateObject(self):
        old_vertices = self.vertices_gnd.copy()
        force, torque = self.calculator.calculateForces(self)
        self.center, self.velocity, self.angle, self.angular_velocity = self.calculator.integrate_motion(
            self.mass,
            self.center,
            self.velocity,
            force,
            self.angular_velocity,
            self.angle,
            torque,
            self.inertia,
            dt=0.01
        )

        self.attitude = self.calculator.rotate(self.attitude, self.angular_velocity * 0.01)

        # self.attitude = self.calculator.rotate(self.attitude, np.pi/6)

        new_vertices_gnd = (self.attitude @ self.vertices_bdy.T).T

        for r in range(len(new_vertices_gnd)):
            new_vertices_gnd[r] += self.center

        #diff = new_vertices_gnd - self.vertices_gnd
        self.vertices_gnd = new_vertices_gnd
        #self.calculateHitbox()

        self.resolveIntersections()
        diff = self.vertices_gnd - old_vertices

        # diff = np.array([[0,0],[0,0],[0,0],[0,0]])
        return diff

