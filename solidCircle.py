import numpy as np

class solidCircle(solidObject):
    """
    Defines some basic properties of the circle
    """
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

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
        print("Child specific")

    def collisionPoint(self, x, y, r):
        """
        Finds the closest point of the boundary and the circle of radius r at point (x,y)
        """
        

    def dist(self, x, y):
        d = np.sqrt((x - self.x)**2 + (y - self.y)**2)
        return d
