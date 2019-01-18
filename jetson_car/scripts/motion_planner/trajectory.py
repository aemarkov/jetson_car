class Trajectory:
    """
    Store trajectory description: position, speed, acceleration of some 
    argument (time, covered arc length, etc)
        y(x), dy/dx(x), d2y/dx^2(x) 
    """
    def __init__(self, x, y, dy, ddy):
        """
        Args:
            y   - list of y(x) points
            dy  - list of dy/dx(x) points
            ddy - list if d2y/dx^2(x) points
            x   - list of x (argument) points
        """
        
        if len(x) != len(dy) or len(dy) != len(ddy) or len(ddy) != len(x):
            raise ValueError('Arrays should be same length')
        
        self.y = y
        self.dy = dy
        self.ddy = ddy
        self.x = x
        self.valid = None
    
    def len(self):
        return len(self.x)