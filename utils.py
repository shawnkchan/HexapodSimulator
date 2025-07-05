from gc import enable
from shapes import Cylinder, Cone
from OpenGL.GL import *
from OpenGL.GLU import *
import imgui

class Axes():
    '''
    Object representing an x, y, and z unit axis
    '''
    def __init__(self):
        self.unitCylinder = Cylinder()
        self.cone = Cone(baseRadius=0.15, tipRadius=0.01, length=0.4)
    
    def draw(self):
        glPushMatrix()

        # z axis, red
        glColor3f(1, 0, 0)
        self.unitCylinder.draw()
        glTranslatef(0, 0, 1)
        self.cone.draw()
        glTranslatef(0, 0, -1)

        # y axis, green
        glRotatef(-90, 1, 0, 0)
        glColor3f(0, 1, 0)
        self.unitCylinder.draw()
        glTranslatef(0, 0, 1)
        self.cone.draw()
        glTranslatef(0, 0, -1)

        # x axis, blue
        glRotatef(90, 0, 1, 0)
        glColor3f(0, 0, 1)
        self.unitCylinder.draw()
        glTranslatef(0, 0, 1)
        self.cone.draw()

        glPopMatrix()

