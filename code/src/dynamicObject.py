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
    
    def resolveIntersections(self, max_iter=5):
        """Resolves overlaps by projecting the object out of collisions."""
        for _ in range(max_iter):
            collision_found = False
            for other in self.calculator.objects:
                if other is self: 
                    continue

                if isinstance(other, Floor):
                    is_touching, info = self.calculator.intersectFloor(self, other)
                else:
                    is_touching, info = self.calculator.intersect(self, other)
                    
                    #is_touching = False
                    
                if is_touching:
                    depth, normal = info[0], info[1]

                    
                    correction = normal * (depth * 0.3)
                    self.center += correction
                    self.vertices_gnd += correction
                    
                    # 2. Velocity Projection: Cancel velocity into the surface
                    v_dot_n = np.dot(self.velocity, normal)
                    if v_dot_n < 0:
                        # This stops the object from 'trying' to stay inside the floor
                        self.velocity -= v_dot_n * normal
                        
                    # 3. Angular Damping:
                    self.angular_velocity *= 1 # 1->bouncy, 0->nothing
                    
                    collision_found = True

            if not collision_found: 
                break

    def updateObject(self):
        timestart = glfw.get_time()
        old_vertices = self.vertices_gnd.copy()
        result = self.calculator.calculateForces(self)
        net_force, net_torque = result[0], result[1]
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

        #diff = new_vertices_gnd - self.vertices_gnd
        self.vertices_gnd = new_vertices_gnd
        #self.calculateHitbox()

        self.resolveIntersections()
        diff = self.vertices_gnd - old_vertices

        # diff = np.array([[0,0],[0,0],[0,0],[0,0]])
        timeend = glfw.get_time()
        print("TIME: ", timeend - timestart)
        return diff

