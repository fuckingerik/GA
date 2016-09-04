import numpy as np
from solidObject import solidObject

class solidCircle(solidObject):
    """
    Defines some basic properties of the circle
    """
    def __init__(self, x, y, r, cons=1):
        self.x = x
        self.y = y
        self.r = r
        self.cons = cons

    def hasCollision(self, x, y, r):
        """
        Checks if a circle of radius r at point (x,y) has collided with the object.
        """
        d = self.dist(x, y)
        if d <= (r + self.r):
            return True
        else:
            return False

    def normCollision(self, x, y):
        """
        return the normal vector of the collision point
        """
        xC = x - self.x
        yC = y - self.y
        mag = np.sqrt(xC**2 + yC**2)
        return (xC/mag, yC/mag)


    def dist(self, x, y):
        d = np.sqrt((x - self.x)**2 + (y - self.y)**2)
        return d
