import numpy as np
from solidObject import solidObject

class solidLine(solidObject):
    """
    Defines some basic properties of a solid line
    """
    def __init__(self, A, B, cons=1):
        """
        A solid line is defined by the two end points
        """
        self.A = A
        self.B = B
        self.cons = cons

    def hasCollision(self, x, y, r):
        """
        Checks if a circle of radius r at point (x,y) has collided with the object.
        """
        #line vector
        x_line = self.B.x - self.A.x
        y_line = self.B.y - self.A.y
        lineDot = x_line**2 + y_line**2
        mag = np.sqrt(x_line**2 + y_line**2)
        #circle vector
        x_circ = x - self.A.x
        y_circ = y - self.A.y
        dot = (x_line*x_circ + y_line*y_circ)/mag
        x_par = x_line*dot/mag
        y_par = y_line*dot/mag
        x_perp = x_circ - x_par
        y_perp = y_circ - y_par
        d = np.sqrt(x_perp**2 + y_perp**2)
        if d <= r and dot >= 0 and dot <= lineDot:
            return True
        else:
            return False

    def normCollision(self, x, y):
        """
        return the normal vector of the collision point
        """
        #line vector
        x_line = self.B.x - self.A.x
        y_line = self.B.y - self.A.y
        mag = np.sqrt(x_line**2 + y_line**2)
        #circle vector
        x_circ = x - self.A.x
        y_circ = y - self.A.y
        dot = (x_line*x_circ + y_line*y_circ)/mag
        x_par = x_line*dot/mag
        y_par = y_line*dot/mag
        x_perp = x_circ - x_par
        y_perp = y_circ - y_par
        mag_perp = np.sqrt(x_perp**2 + y_perp**2)
        return (x_perp/mag_perp, y_perp/mag_perp)

    def dist(self, x, y):
        d = np.sqrt((x - self.x)**2 + (y - self.y)**2)
        return d

    def drawToScreen(self, screen):
        
