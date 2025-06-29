import math as m
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import imgui

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
    pos = [x, y, z, 1]
    finalTransform = np.identity(4, dtype=float)
    for m in transformationMatrices:
        finalTransform @= m
    pos = finalTransform @ pos
    return pos

class ikSolverLeg():
    def __init__(self, origin: dict, name: str, coxaLength, femurLength, tibiaLength):
        self.origin = origin
        self.name = f'{name} Solver'
        self.coxaLength = coxaLength
        self.femurLength = femurLength
        self.tibiaLength = tibiaLength
        self.xGoal = self.coxaLength + self.femurLength + self.tibiaLength
        self.yGoal = 0.0
        self.zGoal = 0.0

    def draw(self):
        if self.xGoal is not None and self.yGoal is not None and self.zGoal is not None:
            goalChanged, vals = imgui.input_float3(f'{self.name} Goal Coordinates', self.xGoal, self.yGoal, self.zGoal)
            self.xGoal = vals[0]
            self.yGoal = vals[1]
            self.zGoal = vals[2]
            glPushMatrix()
            glTranslatef(self.origin['x'], self.origin['y'], self.origin['z']) # translate by offset
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
    
    def coxaAngle(self):
        '''
        Returns the angle of the coxa joint, where positive follows the right hand curl rule wrt the coxa's z-axis

        @param  y    Desired y coordinate for end effector to reach
        @param  x   Desired x coordinate for end effector to reach

        @return coxaAngle   The angle at which the Coxa's joint should be set
        '''
        return m.atan(self.yGoal / self.xGoal)

    def femurAngle(self):
        '''
        Returns the angle of the femur joint, where positive follows the right hand curl rule wrt the femur joint
        '''
        p = self._getRelativeGoalPosition()

        alpha = m.atan2(self.zGoal, p)
        
        theta2 = m.acos(
            (self.tibiaLength**2 - self.femurLength**2 - p**2 - self.zGoal**2) / (-2*self.femurLength * m.sqrt(p**2 + self.zGoal**2))
            )

        femurAngle = theta2 + alpha
        return femurAngle

    def tibiaAngle(self):
        p = self._getRelativeGoalPosition()

        tibiaAngle = -m.acos((p**2 + self.zGoal**2 - self.femurLength**2 - self.tibiaLength**2) / (2 * self.tibiaLength * self.femurLength))

        return tibiaAngle   
    
    def _getRelativeGoalPosition(self):
        '''
        Helper function to get the goal coordinate's position relative to the Coxa joint 
        '''
        xCoxa = self.coxaLength * m.cos(self.coxaAngle())
        yCoxa = self.coxaLength * m.sin(self.coxaAngle())
        p = m.sqrt((self.xGoal - xCoxa)**2 + (self.yGoal - yCoxa)**2)
        return p
    