#Imports
import matplotlib.pyplot as plt
import numpy as np
import random
"""
Learning to throw a basketball using GA
"""

################################################
# Constants
################################################
g = 9.82
xTarget = 14
yTarget = 6

xRing = [13,15]
yRing = [6,6]

################################################
# GA functions
################################################
def fitness(x, y, xv):
    """
    Calcualtes the fitness of a solution
    """
    dist, i = minDist(x, y)
    return dist + abs(xv[i])

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
        xf, yf, xvf =  calcMotion(father[1], father[0], 4, 0.1)
        xm, ym, xvm =  calcMotion(mother[1], mother[0], 4, 0.1)
        fitf = fitness(xf, yf, xvf)
        fitm = fitness(xm, ym, xvm)
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
    stopTime = 4
    timeStep = 0.1

    pop = firstPopulation(0, 15, 0, np.pi/2, popSize)
    for i in range(numGen):
        fits = []
        for j in range(popSize):
            x, y, xv =  calcMotion(pop[j][1], pop[j][0], stopTime, timeStep)
            fits.append(fitness(x, y, xv))
        fits = np.array(fits)
        topI = topIndividuals(fits, bestSize)
        bestPop = []
        for ind in topI:
            bestPop.append(pop[ind])
        xb, yb, xvb = calcMotion(pop[topI[0]][1], pop[topI[0]][0], stopTime, timeStep)
        plt.plot(xb, yb, label="Generation: %d"%i)
        plt.draw()
        pop = formPairs(bestPop, popSize)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.plot(xRing, yRing, label="ring")
    plt.ylim([0,10])
    plt.xlim([0,20])
    plt.show()

        

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
    return v0*np.sin(theta)*t - (g/2)*t**2

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
runGA()
