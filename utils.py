from shapes import Cylinder
from OpenGL.GL import *
from OpenGL.GLU import *

class Axes():
    def __init__(self):
        self.unitCylinder = Cylinder()
    
    def draw(self):
        glPushMatrix()

        # z axis, red
        glColor3f(1, 0, 0)
        self.unitCylinder.draw()

        # x axis, blue
        glRotatef(90, 0, 1, 0)
        glColor3f(0, 0, 1)
        self.unitCylinder.draw()
    
        # y axis, green
        glRotatef(90, 1, 0, 0)
        glColor3f(0, 1, 0)
        self.unitCylinder.draw()

        glPopMatrix()