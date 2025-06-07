from OpenGL.GL import *
from OpenGL.GLU import *
from utils import Axes


class Canvas():
    '''
    Representation of a drawing that gets rendered in a canvas
    '''
    def __init__(self):
        self.mainAxes = Axes()
    
    def drawScene(self, xAngle, yAngle, zAngle):
        glPushMatrix()

        # rotate around the scene
        glRotatef(zAngle, 0, 0, 1)
        glRotatef(xAngle, 1, 0, 0)
        glRotatef(yAngle, 0, 1, 0)

        # draw objects here
        self.mainAxes.draw()

        glPopMatrix()
