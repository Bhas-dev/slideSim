import numpy as np
from src.floor import *

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

    def intersect(self, objA, objB):
        """
        Checks if objA is colliding with objB.
        Returns: (bool, collision_data)
        collision_data = {'depth': float, 'normal': np.array([x, y])}
        """
        max_penetration = -1.0
        best_normal = np.array([0.0, 0.0])
        collision_detected = False

        # We check all vertices of A against the edges of B
        # (assuming B is a convex Polygon)
        for vertex in objA.vertices_gnd:
            for i in range(len(objB.hitbox)):
                # Edge segment defined by points p1 and p2
                p1 = objB.hitbox[i][0]
                p2 = objB.vertices_gnd[i][1]
                
                # edge vector and normal
                edge = p2 - p1
                normal = np.array([-edge[1], edge[0]])
            
                norm_val = np.linalg.norm(normal) # Normalising
                if norm_val == 0: # edge is a point
                    continue
                normal /= norm_val
                
                # Check if the vertex is "inside" the edge

                p1_to_v = vertex - p1
                
                # Dot product gives the signed distance to the line
                # If distance is negative, the point is "inside" the surface
                distance = np.dot(p1_to_v, normal)
                
                if distance < 0:
                    # We found a penetration!
                    penetration = abs(distance)
                    if penetration > max_penetration:
                        max_penetration = penetrationect
                        # The normal should point AWAY from the surface
                        best_normal = normal 
                        collision_detected = True

        if collision_detected:
            return True, [max_penetration, best_normal]
        
        return False, None

    def intersectFloor(self, obj, floor_inst):
        """ normal reaction against floor, we'll be assuming flat floor """
        max_penetration = -0.25
        best_normal = np.array([0.0, 0.0])
        collision_detected = False
        line = floor_inst.hitbox[1]
        print("line : ", line)

        for vertex in obj.vertices_gnd:
            # Edge segment defined by points p1 and p2
            p1 = line[0]
            p2 = line[1]
            
            # edge vector and normal
            edge = p2 - p1
            normal = np.array([-edge[1], edge[0]])
        
            norm_val = np.linalg.norm(normal) # Normalising
            if norm_val == 0: # edge is a point
                continue
            normal /= norm_val
            
            # Check if the vertex is "inside" the edge

            p1_to_v = vertex - p1
            
            # Dot product gives the signed distance to the line
            # If distance is negative, the point is "inside" the surface
            distance = np.dot(p1_to_v, normal)
            
            if distance < 0:
                # We found a penetration!
                penetration = abs(distance)
                if penetration > max_penetration:
                    max_penetration = penetration
                    collision_detected = True

        if collision_detected:
            return True, [max_penetration, normal]
        
        return False, None

    def air_resistance(self, obj):
        drag_coef = 0.47 #spherical object
        air_density = 1.225 #kg/m3
        area = 1.0 #m2, next time get actual value
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
        resulting_force = weight + self.air_resistance(obj) + self.calculateInteractions(obj)# adding all forces
        return resulting_force
    
    def calculateInteractions(self, obj):
        reaction = np.array([.0,.0])
        stiffness = 5000.0 # hardness
        damping = 100.0 # reduce bounce
        for r in range(len(self.objects)):
            if r == obj.calcIdx:
                continue
            other = self.objects[r]
            if isinstance(other, Floor):
                is_touching, collision_info = self.intersectFloor(obj, other)
                if is_touching:
                    print("hiaaa")
                    depth = collision_info[0]
                    normal = collision_info[1]
                    v_rel = obj.velocity
                    v_normal = np.dot(v_rel, normal)
                    force_magnitude = max(0, (stiffness * depth) - (damping * v_normal))
                    reaction += force_magnitude * normal
                continue
            is_touching, collision_info = self.intersect(obj, other)
            if is_touching:
                depth = collision_info[0]
                normal = collision_info[1]

                if other.static:
                    v_rel = obj.velocity
                else:
                    v_rel = obj.velocity - other.velocity
                v_normal = np.dot(v_rel, normal)

                force_magnitude = max(0, (stiffness * depth) - (damping * v_normal))
                reaction += force_magnitude * normal
        return reaction

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