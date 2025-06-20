from OpenGL.GL import *
from OpenGL.GLU import *
from utils import Axes
from parts import Member, Joint, Leg

class Canvas():
    '''
    Representation of the space in which objects gets rendered

    @param hideAxes whether to show or hide the global frame axes
    '''

    def __init__(self, hideAxes=False):
        self.mainAxes = Axes()
        self.hideAxes = hideAxes
        self.objects = [
            # Joint(name='Coxa Joint')
            Leg(name='Leg1', coxaLength=1.5, femurLength=3.0, tibiaLength=5.0, xOrigin=1.0, yOrigin=1.0, zOrigin=1.0)
            ]
    
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

        for j in self.objects:
            j.draw()


        glPopMatrix()
