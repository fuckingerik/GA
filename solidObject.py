"""
Defines a solid object, other object bounces elastically of this
"""
class solidObject:
    """
    Defines some basic properties of the 
    """
    def __init__(self):
        print('Childs must implement this!')

    def hasCollision(self, x, y, r):
        """
        Checks is a circle of radius r at point (x,y) has collided with the object.
        """
        print("Child specific")

    def normCollision(self, x, y):
        """
        return the normal vector of the collision point
        """
        print("Child specific")

    def collisionPoint(self, x, y, r):
        """
        Finds the closest point of the boundary and the circle of radius r at point (x,y)
        """
        print("Child specific")

    def drawToScreen(self, screen):
        """
        Draws object to the screen
        """
        print("Child specific")
