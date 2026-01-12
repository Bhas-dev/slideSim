from src.abstractObject import AbstractObject
import numpy as np
from src.floor import Floor
import glfw

class DynamicObject(AbstractObject):

    def __init__(self, mass, center, calc, velocity = np.array([.0, .0]), attitude = np.array([[1,0],[0,1]])):
        super().__init__(mass, center, calc)
        self.velocity = velocity
        self.angular_velocity = 0.0
        self.attitude = attitude
        self.angle = 0.0
        self.contactobj = None
        self.normal = np.array([0.0, 0.0])
    
    def resolveIntersections(self):
        for other in self.calculator.objects:
            if other is self:
                continue

            if isinstance(other, Floor):
                hit, info = self.calculator.intersectFloor(self, other)
            else:
                hit, info = self.calculator.intersect(self, other)
            if not hit:
                self.contactobj = None
                continue
            depth, normal, contact_pts = info

            self.contactobj = other
            self.normal = normal
            # 1) velocity-level resolution (impulses)
            self.calculator.resolveCollision(self, other, normal, contact_pts)

            # 2) position-level correction
            self.calculator.positionalCorrection(self, normal, depth)

    def updateObject(self):
        old_vertices = self.vertices_gnd.copy()
        force, torque = self.calculator.calculateForces(self)
        self.center, self.velocity, self.angle = self.calculator.integrate_motion(
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
        c = np.cos(self.angle)
        s = np.sin(self.angle)
        self.attitude = np.array([
            [c, -s],
            [s,  c]
        ])

        # self.attitude = self.calculator.rotate(self.attitude, np.pi/6)

        new_vertices_gnd = (self.attitude @ self.vertices_bdy.T).T

        for r in range(len(new_vertices_gnd)):
            new_vertices_gnd[r] += self.center

        #diff = new_vertices_gnd - self.vertices_gnd
        self.vertices_gnd = new_vertices_gnd
        #self.calculateHitbox()

        self.resolveIntersections()
        diff = self.vertices_gnd - old_vertices
        G = 9.81
        force = self.mass * np.array([0.0, -G])
        if self.contactobj!=None and np.linalg.norm(self.velocity) < 0.05:
            normal = self.normal
            tangent = np.array([-normal[1], normal[0]]) # Vecteur le long de la pente
            
            # On calcule la force de gravité projetée sur la pente
            gravity_on_slope = np.dot(force, tangent)
            
            # On calcule la force normale (pression sur le sol)
            N = abs(np.dot(force, normal))
            
            # Limite de friction (Adhésion)
            friction_max = self.calculator.friction_coef(self, self.contactobj) * N
            
            # SI la gravité est plus faible que l'adhésion -> ON BLOQUE
            if abs(gravity_on_slope) <= friction_max:
                # On annule la force parallèle à la pente
                force -= gravity_on_slope * tangent
                # On annule aussi la force qui s'enfonce dans le sol
                force -= np.dot(force, normal) * normal
                
                # Optionnel : si l'objet bouge presque pas, on le stoppe net
                if np.linalg.norm(self.velocity) < 0.1:
                    self.velocity = np.array([0.0, 0.0])
                    self.angular_velocity = 0.0
                    return diff - diff
        # diff = np.array([[0,0],[0,0],[0,0],[0,0]])
        return diff

