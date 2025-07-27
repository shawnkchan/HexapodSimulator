import math as m
from re import X
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import imgui


def dhTransformMatrix(alpha: float, a: float, theta: float, d: float):
    '''
    NOTE: Uses Modified DH, not standard DH
    Returns the homogenous transformation matrix using Denevit-Hartenberg parameters. Used to solve for forward kinematics of kinematic chains.
    Angles are in radians.

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
    Solves for the coordinates of the hexapod leg's tip relative to the base link frame given a list of ordered transformation matrices.
    Each transformation matrix corresponds to a DH transform matrix for a specific configuration of the leg

    @param  x   x-coordinate of the end effector tip relative to the last body frame
    @param  y   y-coordinate of the end effector tip relative to the last body frame
    @param  z   z-coordinate of the end effector tip relative to the last body frame
    @param transformationMatrices   list of the transformation matrices to be applied. Must be in sorted order, ie from the fixed frame to the end effector frame.

    @return pos 4 x 1 matrix representing the x, y, z coordinates of the input coordinates relative to the base link frame
    '''
    pos = [x, y, z, 1]
    finalTransform = np.identity(4, dtype=float)
    for m in transformationMatrices:
        # print(f'final transform: {finalTransform}')
        # print(f'matrix: {m}')
        finalTransform @= m
    pos = finalTransform @ pos
    return pos

class ikSolverLeg():
    def __init__(self, origin: dict, name: str, coxa, femur, tibia):
        self.origin = origin
        self.name = f'{name} Solver'
        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia
        # center the point about the end effector's tip
        self.xGoal = self.coxa.length + self.femur.length + self.tibia.length
        self.yGoal = 0.0
        self.zGoal = 0.0

    def draw(self):
        '''
        Draws the solver's goal point
        '''
        if self.xGoal is not None and self.yGoal is not None and self.zGoal is not None:
            # glPushMatrix()
            # glTranslatef(self.origin['x'], self.origin['y'], self.origin['z']) # translate by offset
            # glTranslatef(self.xGoal, self.yGoal, self.zGoal)
            # quadric = gluNewQuadric()
            # gluQuadricDrawStyle(quadric, GLU_FILL)
            # glColor3f(0, 1, 1)
            # gluSphere(quadric, 0.1, 32, 32) 
            # gluDeleteQuadric(quadric)
            self.drawTrajectoryPoints()
            # glPopMatrix()
    
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
        
        # print((self.tibiaLength**2 - self.femurLength**2 - p**2 - self.zGoal**2))
        # print(-2*self.femurLength * m.sqrt(p**2 + self.zGoal**2))

        temp = max(min((self.tibia.length**2 - self.femur.length**2 - p**2 - self.zGoal**2) / (-2*self.femur.length * m.sqrt(p**2 + self.zGoal**2)), 1), -1)

        theta2 = m.acos(
            temp
            )

        femurAngle = theta2 + alpha
        return femurAngle

    def tibiaAngle(self):
        p = self._getRelativeGoalPosition()

        temp = max(min((p**2 + self.zGoal**2 - self.femur.length**2 - self.tibia.length**2) / (2 * self.tibia.length * self.femur.length), 1), -1)

        tibiaAngle = -m.acos(temp)

        return tibiaAngle   
    
    def _getRelativeGoalPosition(self):
        '''
        Helper function to get the goal coordinate's position relative to the Coxa joint 
        '''
        xCoxa = self.coxa.length * m.cos(self.coxaAngle())
        yCoxa = self.coxa.length * m.sin(self.coxaAngle())
        p = m.sqrt((self.xGoal - xCoxa)**2 + (self.yGoal - yCoxa)**2)
        return p
    
    def createLegTrajectory(self, xStart, yStart, zStart):
        '''
        Creates a list of points in space for leg motion
        '''
        Y_DISPLACEMENT = -4.0
        Z_DISPLACEMENT = 2.0

        start = (xStart, yStart, zStart)
        end = (xStart, yStart + Y_DISPLACEMENT, 0)

        peak = (xStart, yStart + Y_DISPLACEMENT / 2, Z_DISPLACEMENT)

        return [start, peak, end]

    def drawTrajectoryPoints(self):
        glColor3f(0, 1, 1)
        glPointSize(9.0)
        glBegin(GL_POINTS)
        for p in self.createLegTrajectory(5.0, 0.0, -2.0):
            glVertex3f(p[0], p[1], p[2])
        glEnd()


