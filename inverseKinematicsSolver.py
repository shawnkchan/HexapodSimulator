import math
import imgui
from OpenGL.GL import *
from OpenGL.GLU import *

def coxaAngle(y, x):
    '''
    Returns the angle of the coxa joint, where positive follows the right hand curl rule wrt the coxa's z-axis

    @param  y    Desired y coordinate for end effector to reach
    @param  x   Desired x coordinate for end effector to reach

    @return coxaAngle   The angle at which the Coxa's joint should be set
    '''
    return math.atan(y / x)

def femurAngle(femurLength, tibiaLength, x, y, z):
    '''
    Returns the angle of the femur joint, where positive follows the right hand curl rule wrt the femur joint
    '''
    femurAngle = math.acos(
        (tibiaLength**2 - femurLength**2 - x**2 - y**2 - z**2) / (-2*tibiaLength * math.sqrt(x**2 + y**2 + z**2))
        ) + math.atan(
            z / math.sqrt(x **2 + y**2)
            )

    return femurAngle

def tibiaAngle(femurLength, tibiaLength, x, y, z):
    tibiaAngle = math.acos((x**2 + y**2 + z**2 - femurLength**2 - tibiaLength**2) / (2 * tibiaLength * femurLength))

    return tibiaAngle

def minimumJointAngle(linkHeight, jointRadius):
    gamma = math.atan((linkHeight / 2) / (jointRadius))
    return 2 * gamma


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
    