import math as m
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np

def coxaAngle(y, x):
    '''
    Returns the angle of the coxa joint, where positive follows the right hand curl rule wrt the coxa's z-axis

    @param  y    Desired y coordinate for end effector to reach
    @param  x   Desired x coordinate for end effector to reach

    @return coxaAngle   The angle at which the Coxa's joint should be set
    '''
    return m.atan(y / x)

def femurAngle(femurLength, tibiaLength, x, y, z):
    '''
    Returns the angle of the femur joint, where positive follows the right hand curl rule wrt the femur joint
    '''
    femurAngle = m.acos(
        (tibiaLength**2 - femurLength**2 - x**2 - y**2 - z**2) / (-2*tibiaLength * m.sqrt(x**2 + y**2 + z**2))
        ) + m.atan(
            z / m.sqrt(x **2 + y**2)
            )

    return femurAngle

def tibiaAngle(femurLength, tibiaLength, x, y, z):
    tibiaAngle = m.acos((x**2 + y**2 + z**2 - femurLength**2 - tibiaLength**2) / (2 * tibiaLength * femurLength))

    return tibiaAngle

def minimumJointAngle(linkHeight, jointRadius):
    gamma = m.atan((linkHeight / 2) / (jointRadius))
    return 2 * gamma

def dhTransformMatrix(alpha: float, a: float, theta: float, d: float):
    '''
    returns the homogenous transformation matrix using Denevit-Hartenberg parameters

    @param  alpha   link twist
    @param  a   link length
    @param theta    joint angle
    @param d    link offset
    '''
    matrix = [
        [m.cos(theta), -m.sin(theta), 0, a],
        [m.cos(alpha) * m.sin(theta), m.cos(alpha) * m.cos(theta), -m.sin(alpha), -d * m.sin(alpha)],
        [m.sin(alpha) * m.sin(theta), m.sin(alpha) * m.cos(theta), m.cos(alpha), d * m.cos(alpha)],
        [0, 0, 0, 1]
    ]
    return  matrix

def endEffectorPosition(x: float, y: float, z: float, transformationMatrices: list[list[list]]):
    '''
    solves for the end effector position of the hexapod leg given a list of ordered transformation matrices

    @param  x   x-coordinate of the end effector relative to the last body frame
    @param  y   y-coordinate of the end effector relative to the last body frame
    @param  z   z-coordinate of the end effector relative to the last body frame
    @param transformationMatrices   list of the transformation matrices to be applied. Must be in sorted order, ie from the fixed frame to the last body frame
    '''
    pos = [x, y, x, 1]
    finalTransform = np.identity(4, dtype=float)
    for m in transformationMatrices:
        finalTransform @= m
    pos = finalTransform @ pos
    return pos

class ikSolver():
    def __init__(self, origin: dict, name: str):
        self.origin = origin
        self.name = f'{name} Solver'
        self.xGoal = None
        self.yGoal = None
        self.zGoal = None

    def draw(self):
        if self.xGoal and self.yGoal and self.zGoal:
            glPushMatrix()
            glTranslatef(self.xGoal, self.yGoal, self.zGoal)
            quadric = gluNewQuadric()
            gluQuadricDrawStyle(quadric, GLU_FILL)
            glColor3f(0, 1, 1)
            gluSphere(quadric, 0.1, 32, 32) 
            gluDeleteQuadric(quadric)
            glPopMatrix()
    
    def setGoalCoordinates(self, x, y, z):
        self.xGoal = x
        self.yGoal = y
        self.zGoal = z
    