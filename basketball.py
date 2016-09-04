#Imports
import matplotlib.pyplot as plt
import numpy as np
import random
from solidCircle import solidCircle
from solidLine import solidLine
from vectorMath import Vec
from vectorMath import Point
from ball import Ball

"""
Learning to throw a basketball using GA
"""

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

timeStep = 0.01
solidObjects = []
solidObjects.append(solidCircle(xRing[0], yRing[0], thickRad, cons = 0.85))
solidObjects.append(solidCircle(xRing[1], yRing[1], thickRad, cons = 0.95))
solidObjects.append(solidLine(Point(xBack[0], yBack[0]), Point(xBack[1], yBack[1]), cons = 0.90))
solidObjects.append(solidLine(Point(xCon[0], yCon[0]), Point(xCon[1], yCon[1]), cons = 0.95))
solidObjects.append(solidLine(Point(0, -throwHeight), Point(5, -throwHeight), cons=0.8))

################################################
# GA functions
################################################
def fitness(x, y, xv, yv):
    """
    Calcualtes the fitness of a solution
    """
    dist, i = minDist(x, y)
    return dist*2 + abs(xv[i])*dist + yv[i]/2

def minDist(x, y):
    dists  = [0]*len(x)
    for i in range(len(x)):
        dists[i] =  np.sqrt((x[i] - xTarget)**2 + (y[i] - yTarget)**2)
    minD = min(dists)
    minI = dists.index(minD)
    return minD, minI

def firstPopulation(lowV, highV, lowA, highA, num):
    """
    Creates a first population
    """
    pop = [[] for _ in range(num)]
    for i in range(num):
        pop[i] = (np.random.uniform(low=lowA, high=highA), np.random.uniform(low=lowV, high=highV))
    return pop

def topIndividuals(popFit, num):
    """
    return the index of the best individuals
    """
    indSorted = np.argsort(popFit)
    return indSorted[:num]

def formPairs(pop, num):
    """
    Forms num of pairs from the population by sampling with replacements
    """
    newPop = []
    for i in range(num):
        father = random.choice(pop)
        mother = random.choice(pop) #Triggered!!!!!!!
        xf, yf, xvf, yvf =  calculatePositions(father[1], father[0], 300, 0.01)
        xm, ym, xvm, yvm =  calculatePositions(mother[1], mother[0], 300, 0.01)
        #xf, yf, xvf =  calcMotion(father[1], father[0], 4, 0.1)
        #xm, ym, xvm =  calcMotion(mother[1], mother[0], 4, 0.1)
        fitf = fitness(xf, yf, xvf, yvf)
        fitm = fitness(xm, ym, xvm, yvm)
        child = childFromParents(father, mother, fitf, fitm)
        newPop.append(child)
    return newPop

def childFromParents(father, mother, fitf, fitm):
    """

    """
    totFit = fitf + fitm
    
    angle = (father[0]*fitm + mother[0]*fitf)/totFit
    velocity = (father[1]*fitm + mother[1]*fitf)/totFit
    return(angle, velocity)

def runGA():
    popSize = 100
    bestSize = 20
    numGen = 10
    betsPerGen = []
    stopTime = 300
    timeStep = 0.01

    pop = firstPopulation(0, 13, 0, np.pi/2, popSize)
    for i in range(numGen):
        fits = []
        plt.figure(num=i, figsize=(20, 25), dpi=80, facecolor='w', edgecolor='k')
        for j in range(popSize):
            #x, y, xv =  calcMotion(pop[j][1], pop[j][0], stopTime, timeStep)
            x, y, xv, yv = calculatePositions(pop[j][1], pop[j][0], stopTime, timeStep)
            plt.plot(x,y,'b--')
            plt.draw()
            fits.append(fitness(x, y, xv, yv))
        fits = np.array(fits)
        topI = topIndividuals(fits, bestSize)
        bestPop = []
        for ind in topI:
            bestPop.append(pop[ind])
        xb, yb, xvb, yvb = calculatePositions(pop[topI[0]][1], pop[topI[0]][0], stopTime, timeStep)
        #xb, yb, xvb = calcMotion(pop[topI[0]][1], pop[topI[0]][0], stopTime, timeStep)
        plt.plot(xb, yb, 'r--', label='Best. Angle = %f, Vel = %f'%(pop[topI[0]][0], pop[topI[0]][1]))
        plt.draw()
        pop = formPairs(bestPop, popSize)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        xCirc, yCirc = drawCircle(xRing[0], yRing[0], thickRad)
        xCirc2, yCirc2 = drawCircle(xRing[1], yRing[1], thickRad)
        plt.plot(xBack, yBack, 'r')
        plt.plot(xCon, yCon, 'r')
        plt.plot(xCirc, yCirc, 'r')
        plt.plot(xCirc2, yCirc2, 'r')
        plt.plot([0, -throwHeight], [5, -throwHeight], 'r')
        plt.xlim((0,5))
        plt.ylim(-throwHeight,5)
        plt.legend()
        plt.show()

def handleCollision(vel, norm, cons=1):
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
    new_vX = (vP_x + xC*abs(dot))*np.sqrt(cons)
    new_vY = (vP_y + yC*abs(dot))*np.sqrt(cons)
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

################################################
# Position functions
################################################
def xPos(t, v0, theta):
    """
    The x position of the projectile after time t,
    """
    return v0*np.cos(theta)*t

def xVel(t, v0, theta):
    """
    The x velocity of the ball
    Will be used later when collisions are possible
    """
    return v0*np.cos(theta)

def yPos(t, v0, theta):
    """
    The y position of the projectile after time t,
    """
    return v0*np.sin(theta)*t + (g/2)*t**2

def calcMotion(v0, theta, stopTime, timeStep):
    """
    Calculates the motion of the basketball
    """
    tVec = np.arange(0, stopTime, timeStep)
    xPosition = []
    yPosition = []
    xVelocity = []
    for t in tVec:
        xPosition.append(xPos(t, v0, theta))
        yPosition.append(yPos(t, v0, theta))
        xVelocity.append(xVel(t, v0, theta))
    return(xPosition, yPosition, xVelocity)

def checkCollision(objectList):
    """
    Checks for colission between the ball and other objects
    """
    

def run():
    """
    runs the experiment
    """
    xPos1, yPos1, xVel1 = calcMotion(10, np.pi/4, 3, 0.1)
    xPos2, yPos2, xVel2 = calcMotion(15, np.pi/3, 3, 0.1)
    print(fitness(xPos1, yPos1, xVel1))
    print(fitness(xPos2, yPos2, xVel2))
    plt.plot(xPos1, yPos1, 'r--', label="fast")
    plt.plot(xPos2, yPos2, 'b--', label="slow")
    plt.plot(xRing, yRing, label="ring")
    plt.legend()
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()
#run()

def calculatePositions(vel, angle, stopTime, timeStep):
    """
    Calculates the motion given the initial velocity and angle
    """
    xPos = []
    yPos = []
    xVel = []
    yVel = []
    ball = Ball(Point(0,0), Vec(angle, vel), rad)
    for i in range(stopTime):
        ball.updatePos(timeStep)
        xPos.append(ball.pos.x)
        yPos.append(ball.pos.y)
        xVel.append(ball.vel.xComp())
        yVel.append(ball.vel.yComp())
        collided = False
        for j in range(len(solidObjects)):
            if solidObjects[j].hasCollision(ball.pos.x, ball.pos.y, ball.rad):
                (xC, yC) = solidObjects[j].normCollision(ball.pos.x, ball.pos.y)
                vX, vY = handleCollision(ball.vel, [xC, yC], solidObjects[j].cons)
                ball.setVel(vX, vY)    
                collided = True
        if not collided:
            ball.updateVel(timeStep, 0, g)
    return(xPos, yPos, xVel, yVel)
runGA()
