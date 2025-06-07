from OpenGL.GL import *
from OpenGL.GLU import *
from utils import Axes
from parts import Joint

class Canvas():
    '''
    Representation of the space in which a scene gets rendered
    '''
    def __init__(self):
        self.mainAxes = Axes()
        self.hideAxes = False
    
    def drawScene(self, xAngle, yAngle, zAngle, zoomVal):
        glPushMatrix()

        # rotate around the scene
        glRotatef(zAngle, 0, 0, 1)
        glRotatef(xAngle, 1, 0, 0)
        glRotatef(yAngle, 0, 1, 0)
        glTranslatef(zoomVal, zoomVal, zoomVal)

        # draw objects here
        if not self.hideAxes:
            self.mainAxes.draw()
        
        joint = Joint(color=glColor3f(0, 0, 0), length=32)
        joint.draw()

        glPopMatrix()
