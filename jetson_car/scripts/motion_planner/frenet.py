import numpy as np

MIN_POINT_DIFF = 0.05

class FrenetFrame:
    """
    Represents the Frenet Frame
    
    Perform transformation world frame -> local frenet frame
    and local frenet frame -> world frame
    
    Notation:        
        tr  - tangent vector
        nr  - normal vector
        br  - binormal vector (for surface - up)
        Rwf - orientation of local frame with respect to parent frame           
        Rfw - orientation of world frame with respect to parent frame
        Pwf - position of the origin of the local with respect to parent frame
        S0  - covered curve length at the origin of the current frame
    
    Math:
        local->parent transformation:
            Qw = Pwf + Rwf*Qf
            Qa - position in parent frame
            Qb - position in local frame
    
    """
    def __init__(self, s0, p1, p2):
        """
        Create Frenet Frame with two close points on discrete curve.
        Frenet Frame origin will be in point p1
        
        Args:
            s0     - Covered curve length at the point p1.
                     (sum of the length of the all previous segments)
            p1, p2 - Current and next points on the curve.
                     p1p2 is a tangent vector.             
        """
        # local frame axis in parent frame
        #print('(%0.2f, %0.2f), (%0.2f, %0.2f)' % (p1[0], p1[1], p2[0], p2[1]))
        self.br = [0, 0, 1]
        self.tr = p2-p1
        self.tr = self.tr/np.linalg.norm(self.tr)
        self.nr = np.cross(self.br, self.tr)[:2]
        
        # local frame orientation with respect to parent frame
        self.Rwf = np.identity(2)
        self.Rwf[:,0]=self.tr
        self.Rwf[:,1]=self.nr
        #self.Rwf[:,2]=self.br
        
        # parent frame orientation with respect to local frame
        self.Rfw = np.transpose(self.Rwf)
        
        # position of the origin of the local with respect to parent frame
        self.Pwf = p1
        
        self.S0 = np.array([s0, 0])
        
    def point_to(self, point):
        """
        Transofrm point in parent frame to the local frame
        """
        return np.matmul(self.Rfw, point - self.Pwf) + self.S0
        
    def point_from(self, point):
        """
        Transofrm point in local frame to the parent frame
        """
        return np.matmul(self.Rwf, point - self.S0)+self.Pwf
    
    def vector_to(self, vector):
        """
        Transform the vector in parent frame to the local frame
        (Position of the frame origin has no effect)
        """
        return np.matmul(self.Rfw, vector)
    
    def vector_from(self, vector):
        """
        Transform the vector in local frame to the parent frame
        (Position of the frame origin has no effect)
        """
        return np.matmul(self.Rwf, vector)


def path_to_global(local_trajectory, trajectory):
    """
    Convert local path in Frenet Frame to the path in global frame
    
    Args:
        local_trajectory  - local trajectory in Frenet Frame (numpy array)
        trajectory        - original (reference) trajectory. 
                            local path will be "bended" along trajectory
    """
    global_trajectory = np.zeros((len(local_trajectory), 2))
    trajectory_index = 0
    trajectory_s = 0
    
    for i, point in enumerate(local_trajectory):
        s = point[0]
        
        # Find index of the point on curve with s (covered length)
        # more then S of the current point in frenet frame
        while True:

            if trajectory_index >= len(trajectory)-1:
                return global_trajectory[:i]

            segment_len =  np.linalg.norm(trajectory[trajectory_index+1] - trajectory[trajectory_index])
            if trajectory_s + segment_len >= s:
                break

            trajectory_s += segment_len
            trajectory_index += 1
            
        p1 = trajectory[trajectory_index]
        p2 = trajectory[trajectory_index+1]                        
        
        frenet = FrenetFrame(trajectory_s, p1, p2)
        p = frenet.point_from(point)        
        global_trajectory[i] = frenet.point_from(point)
        
    return global_trajectory
