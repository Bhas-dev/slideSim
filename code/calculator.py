import numpy as np
from src.floor import *
from src.ramp import *

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
        coefA = obj1.adhesion_coeff 
        coefB = obj2.adhesion_coeff
        if coefA<0 or coefB<0:
            return 0.0
        return (np.sqrt(coefA * coefB))
    
    def friction_force(self, obj, friction_coef):
        """ Calculates the friction force acting on the object."""
        norm_vel = np.linalg.norm(obj.velocity)
        if norm_vel < 1e-5:
            return np.array([0.0, 0.0])

        angle = np.arctan2(obj.attitude[1][0], obj.attitude[0][0]) 
        if angle<0 :
            angle+= np.pi/2 # angle of the normal to the surface
        return (- friction_coef * obj.mass * G * np.cos(angle) * obj.velocity/norm_vel)

    def intersect(self, objA, objB):
        """
        Checks collision between two objects with multiple convex sub-polygons.
        hitbox format: [sub_poly1, sub_poly2, ...] where sub_poly is [idx1, idx2, idx3]
        """
        max_penetration = -1e9
        best_normal = np.array([0.0, 0.0])
        collision_detected = False
        all_contact_pts = []

        for subA_indices in [[edge[0] for edge in part] for part in objA.hitbox]:
            vertsA = [objA.vertices_gnd[i] for i in subA_indices]
            
            for subB_indices in [[edge[0] for edge in part] for part in objB.hitbox]:
                vertsB = [objB.vertices_gnd[i] for i in subB_indices]
                # 1. Check vertices of A against edges of B
                hit_A_in_B, info_A = self.check_collision(vertsA, vertsB, False)
                if hit_A_in_B:
                    collision_detected = True
                    all_contact_pts.extend(info_A[2])
                    if info_A[1][1] > best_normal[1] or (info_A[0] > max_penetration and info_A[1][1] >= best_normal[1]):
                        max_penetration, best_normal = info_A[0], info_A[1]

                # 2. Check vertices of B against edges of A (Prevents falling through corners)
                hit_B_in_A, info_B = self.check_collision(vertsB, vertsA, True)
                if hit_B_in_A:
                    collision_detected = True   
                    all_contact_pts.extend(info_B[2])
                    curr_normal = -info_B[1]
                    if curr_normal[1] > best_normal[1] + 0.01:
                        max_penetration = info_B[0]
                        best_normal = curr_normal
                    elif abs(curr_normal[1] - best_normal[1]) < 0.01 and info_B[0] > max_penetration:
                        max_penetration = info_B[0]
                        best_normal = curr_normal

        if collision_detected:
            # Deduplicate points
            unique_pts = list({tuple(p): p for p in all_contact_pts}.values())
            return True, [max_penetration, best_normal, unique_pts]
        
        return False, None

    def intersectFloor(self, obj, floor_inst):
        """ normal reaction against floor, we'll be assuming flat floor """
        max_penetration = -0.0001
        best_normal = np.array([0.0, 0.0])
        collision_detected = False
        contact_points = []
        line = floor_inst.vertices_gnd[floor_inst.hitbox[0][1]]

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
            return True, [max_penetration, normal, contact_points]
        
        return False, None

    def check_collision(self, vertsA, vertsB, isB):
        """Checks if any vertex of convex A is inside convex B"""
        deepest_depth = -1e9
        best_normal = np.array([0.0, 0.0])
        points = []
        
        # Check all vertices of A
        for v in vertsA:
            is_inside = True
            min_v_depth = float('inf')
            v_normal = np.array([0.0, 0.0])
            
            # Check against all edges of B
            for i in range(len(vertsB)):
                p1 = vertsB[i]
                p2 = vertsB[(i + 1) % len(vertsB)]
                
                edge = p2 - p1
                # Outward facing normal (Assumes CCW winding)
                normal = np.array([-edge[1], edge[0]])
                norm_len = np.linalg.norm(normal)
                if norm_len < 1e-9: continue
                normal /= norm_len
                
                # Distance from point to edge
                dist = np.dot(v - p1, normal)
                
                if dist > 0.001: # Point is outside this edge (with small epsilon)
                    is_inside = False
                    break
                else:
                    # Track the shallowest penetration to find the exit vector
                    if abs(dist) < min_v_depth: # and normal[1] > 0:
                        #if isB or (not isB and normal[1] > 0):
                        if normal[1]>0:
                            min_v_depth = abs(dist)
                            v_normal = normal # Direction to push A out of B

            if is_inside:
                points.append(v)
                if min_v_depth > deepest_depth:
                    deepest_depth = min_v_depth
                    best_normal = v_normal
                    
        if points:
            return True, [deepest_depth, best_normal, points]
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
        force = obj.mass * np.array([0.0, -G])
        force += self.air_resistance(obj)

        return force, .0
    
    def resolveCollision(self, obj, other, normal, contact_pts):
        if obj.mass == 0:
            return

        inv_mass = 1.0 / obj.mass
        inv_inertia = 1.0 / obj.inertia

        restitution = obj.restitution
        friction = self.friction_coef(obj, other)

        for pt in contact_pts:
            r = pt - obj.center

            # velocity at contact point
            v_contact = obj.velocity + self.perp(r) * obj.angular_velocity
            v_n = np.dot(v_contact, normal)

            if v_n >= -0.01:
                continue  # separating

            rn = self.cross2(r, normal)
            denom = inv_mass + (rn * rn) * inv_inertia

            # --- normal impulse ---
            Jn = -(1 + restitution) * v_n / denom
            impulse_n = Jn * normal

            obj.velocity += impulse_n * inv_mass
            obj.angular_velocity += self.cross2(r, impulse_n) * inv_inertia

            # --- friction impulse ---
            v_contact = obj.velocity + self.perp(r) * obj.angular_velocity
            tangent = v_contact - np.dot(v_contact, normal) * normal

            speed = np.linalg.norm(tangent)
            if speed < 1e-6:
                continue

            tangent /= speed
            rt = self.cross2(r, tangent)
            denom_t = inv_mass + (rt * rt) * inv_inertia

            Jt = -np.dot(v_contact, tangent) / denom_t
            Jt = np.clip(Jt, -friction * Jn, friction * Jn)

            impulse_t = Jt * tangent

            obj.velocity += impulse_t * inv_mass
            obj.angular_velocity += self.cross2(r, impulse_t) * inv_inertia
    
    def positionalCorrection(self, obj, normal, depth):
        if obj.mass == 0:
            return

        percent = 0.2 
        slop = 0.01
        correction = normal * depth
        obj.center += correction
        obj.vertices_gnd += correction

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

    @staticmethod
    def cross2(a, b):
        return a[0] * b[1] - a[1] * b[0]

    @staticmethod
    def perp(v):
        return np.array([-v[1], v[0]])