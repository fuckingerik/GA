from vectorMath import Vec
from vectorMath import Point
from solidCircle import solidCircle
from solidLine import solidLine
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


    def setVel(self, xVel, yVel):
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

    ################################################
    # Constants
    ################################################
    g = -9.82
    throwHeight = 2.5
    xTarget = 4.185
    yTarget = 3.048 - throwHeight
    
    #Ring meassurements
    xRing = [3.96, 4.41]
    yRing = [3.048 - throwHeight, 3.048 - throwHeight]
    ringRad = 0.225
    thickRad = 0.0085
    
    #backboard
    xBack = [4.50, 4.50]
    yBack = [2.95 - throwHeight, 4.05 - throwHeight]
    xCon = [4.41, 4.50]
    yCon = [3.048 - throwHeight, 3.048 - throwHeight]
    
    #baskettball
    rad = 0.125
    #################################################
    xPos = []
    yPos = []
    ball = Ball(Point(0,0), Vec(np.pi/2.5, 8.9), rad)
    timeStep = 0.01
    solidObjects = []
    solidObjects.append(solidCircle(xRing[0], yRing[0], thickRad))
    solidObjects.append(solidCircle(xRing[1], yRing[1], thickRad))
    solidObjects.append(solidLine(Point(xBack[0], yBack[0]), Point(xBack[1], yBack[1])))
    solidObjects.append(solidLine(Point(xCon[0], yCon[0]), Point(xCon[1], yCon[1])))
    solidObjects.append(solidLine(Point(0, -throwHeight), Point(5, -throwHeight)))
    for i in range(300):
        ball.updatePos(timeStep)
        xPos.append(ball.pos.x)
        yPos.append(ball.pos.y)
        collided = False
        for j in range(len(solidObjects)):
            if solidObjects[j].hasCollision(ball.pos.x, ball.pos.y, ball.rad):
                (xC, yC) = solidObjects[j].normCollision(ball.pos.x, ball.pos.y)
                print(xC, yC)
                vX, vY = handleCollision(ball.vel, [xC, yC])
                ball.setVel(vX, vY)    
                collided = True
        if not collided:
            ball.updateVel(timeStep, 0, -9.82)
            
    plt.plot(xPos, yPos)
    xCirc, yCirc = drawCircle(xRing[0], yRing[0], thickRad)
    xCirc2, yCirc2 = drawCircle(xRing[1], yRing[1], thickRad)
    plt.plot(xBack, yBack, 'r')
    plt.plot(xCon, yCon, 'r')
    plt.plot(xCirc, yCirc, 'r')
    plt.plot(xCirc2, yCirc2, 'r')
    plt.plot([0, -throwHeight], [5, -throwHeight], 'r')
    plt.xlim((0,5))
    plt.ylim(-throwHeight,5)
    plt.show()

def handleCollision(vel, norm):
    """
    Computes new velocity vector after a collision
    """
    xC = norm[0]
    yC = norm[1]
    vX = vel.xComp()
    vY = vel.yComp()
    dot = xC*vX + yC*vY
    vN_x = xC*dot
    vN_y = yC*dot
    vP_x = vX - vN_x
    vP_y = vY - vN_y
    new_vX = vP_x + xC*abs(dot)
    new_vY = vP_y + yC*abs(dot)
    return(new_vX, new_vY)

def drawCircle(x, y, rad, points=100):
    """
    Return coordinates of a circle
    """
    xPos = [0]*points
    yPos = [0]*points
    for i in range(points):
        ang = 2*np.pi*i/points
        xPos[i] = rad*np.cos(ang) + x
        yPos[i] = rad*np.sin(ang) + y
    return (xPos, yPos)
#run()
