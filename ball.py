from vectorMath import Vec
from vectorMath import Point
import matplotlib.pyplot as plt
import numpy as np
class Ball:
    """
    Defines a ball
    """
    def __init__(self, pos, vel, rad):
        self.pos = pos
        self.vel = vel
        self.rad = rad

    def updatePos(self, timeStep):
        """
        updates the position of the ball
        """
        xVel = self.vel.xComp()
        yVel = self.vel.yComp()
        self.pos.x += xVel*timeStep
        self.pos.y += yVel*timeStep
        
    def updateVel(self, timeStep, xAcc, yAcc):
        """
        Updates the velocity of the ball
        """
        xVel = self.vel.xComp() + xAcc*timeStep
        yVel = self.vel.yComp() + yAcc*timeStep
        mag = np.sqrt(xVel**2 + yVel**2)
        if (xVel >= 0 and yVel >= 0): #first quadrant
            self.vel.angle = np.arccos(xVel/mag)
        elif (xVel < 0 and yVel >= 0): #seconf quadrant
            self.vel.angle = np.arccos(yVel/mag) + np.pi/2
        elif(xVel < 0 and yVel < 0): #third quad
            self.vel.angle = np.arccos(abs(xVel)/mag) + np.pi
        else: #forth quad
            self.vel.angle = np.arccos(abs(yVel)/mag) + 1.5*np.pi
        self.vel.mag = mag


def run():
    xPos = []
    yPos = []
    ball = Ball(Point(0,0), Vec(np.pi/4, 10), 0.45)
    timeStep = 0.1
    for i in range(50):
        ball.updatePos(timeStep)
        xPos.append(ball.pos.x)
        yPos.append(ball.pos.y)
        ball.updateVel(timeStep, 0, -9.82)
    
    plt.plot(xPos, yPos)
    plt.show()
run()
