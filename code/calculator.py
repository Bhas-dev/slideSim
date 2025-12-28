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
        """ Calculates the gravitational force acting on the object."""
        return (obj.mass * G) * np.array([0, -1]) #m*g*vect(-y)
    
    def friction_coef(self, obj1, obj2):
        """ Calculates the friction coefficient between two objects."""
        return (0.05 + 0.8 * min(obj1.adhesion_coeff, obj2.adhesion_coeff))
    
    def friction_force(self, obj, friction_coef):
        """ Calculates the friction force acting on the object."""
        angle = np.arctan2(obj.attitude[1][0], obj.attitude[0][0])
        return (- friction_coef * obj.mass * G * np.cos(angle) * obj.velocity/np.linalg.norm(obj.velocity))

    def intersect(self, objA, objB):
        """
        Checks if objA is colliding with objB.
        Returns: (bool, collision_data)
        collision_data = {'depth': float, 'normal': np.array([x, y])}
        """
        max_penetration = -0.0001
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
                        max_penetration = penetration
                        # The normal should point AWAY from the surface
                        best_normal = normal 
                        collision_detected = True

        if collision_detected:
            return True, [max_penetration, best_normal]
        
        return False, None

    def intersectFloor(self, obj, floor_inst):
        """ normal reaction against floor, we'll be assuming flat floor """
        max_penetration = -0.0001
        best_normal = np.array([0.0, 0.0])
        collision_detected = False
        contact_points = []
        line = floor_inst.hitbox[1]

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
            
            if distance <= 0:
                # We found a penetration!
                penetration = abs(distance)
                if penetration >= max_penetration:
                    max_penetration = penetration
                    collision_detected = True
                    contact_points.append(vertex)

        if collision_detected:
            print("THIS IS IT RIGHT NOWWWWWWWWWW")
            return True, [max_penetration, normal, contact_points]
        else:
            print()
        
        return False, None

    def air_resistance(self, obj):
        """ Calculates the air resistance (drag) force acting on the object."""
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
        interactions = self.calculateInteractions(obj)
        resulting_force = weight + self.air_resistance(obj) + interactions[0] # adding all forces
        resulting_torque = interactions[1]
        return resulting_force, resulting_torque
    
    def calculateInteractions(self, obj):
        reaction_force = np.array([.0,.0])
        reaction_torque = 0.0
        stiffness = 5000.0 # hardness
        damping = 100.0 # reduce bounce
        for r in range(len(self.objects)):
            if r == obj.calcIdx:
                continue
            other = self.objects[r]
            if isinstance(other, Floor):
                is_touching, collision_info = self.intersectFloor(obj, other)
                if is_touching:
                    depth = collision_info[0]
                    normal = collision_info[1]
                    contact_pts = collision_info[2]
                    for contact_pt in contact_pts:
                        r = contact_pt - obj.center

                        v_rot = np.array([-obj.angular_velocity * r[1], obj.angular_velocity * r[0]])
                        v_rel = obj.velocity + v_rot
                        v_normal = np.dot(v_rel, normal)
                        force_mag = max(0, (stiffness * depth) - (damping * v_normal))
                        print(force_mag)
                        f_vec = force_mag * normal /len(contact_pts)
                        
                        torque = r[0] * f_vec[1] - r[1] * f_vec[0]
                
                        reaction_force += f_vec
                        reaction_torque += torque
                        
                continue
            is_touching, collision_info = self.intersect(obj, other)
            if is_touching:
                depth = collision_info[0]
                normal = collision_info[1]
                contact_pts = collision_info[2]
                for contact_pt in contact_pts:
                        r = contact_pt - obj.center

                        v_rot = np.array([-obj.angular_velocity * r[1], obj.angular_velocity * r[0]])
                        if other.static:
                            v_rel = obj.velocity + v_rot
                        else:
                            v_rel = obj.velocity + v_rot - other.velocity
                        v_normal = np.dot(v_rel, normal)
                        force_mag = max(0, (stiffness * depth) - (damping * v_normal))
                        f_vec = force_mag * normal /contact_pts.shape[0]
                        
                        torque = r[0] * f_vec[1] - r[1] * f_vec[0]
                
                        reaction_force += f_vec
                        reaction_torque += torque

        return reaction_force, reaction_torque

    @staticmethod
    def integrate_motion(mass, current_position, current_velocity, net_force, current_angular_vel, current_angle, net_torque, inertia, dt):
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

        # 4. Calculate Angular Acceleration (alpha = torque / I)
        angular_accel = net_torque / inertia

        # 5. Update Angular Velocity and Angle
        new_angular_velocity = current_angular_vel + angular_accel * dt
        new_angle = current_angle + new_angular_velocity * dt
        
        return new_position, new_velocity, new_angle, new_angular_velocity