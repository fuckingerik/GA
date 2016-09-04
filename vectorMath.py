import numpy as np

"""
Defines function used to make vector calculations
and defines some basic shapes
"""

class Vec:

    def __init__(self, angle, mag):
        self.angle = angle
        self.mag = mag

    def xComp(self):
        """
        return the magnitude in x direction (1,0)
        """
        return np.cos(self.angle)*self.mag

    def yComp(self):
        """
        return the magnitude in y direction (0,1)
        """
        return np.sin(self.angle)*self.mag

class Point:
    """
    Defines a point
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
