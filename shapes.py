from OpenGL.GL import *
from OpenGL.GLU import *

class Cylinder():
    def __init__(self, base=0.1, top=0.1, length=1.0, slices=32):
        self.base = base
        self.top = top
        self.length = length
        self.slices = slices

    def draw(self):
        quadric = gluNewQuadric()
        gluQuadricDrawStyle(quadric, GLU_FILL)
        gluCylinder(quadric, self.base, self.top, self.length, self.slices, 1)
        gluDeleteQuadric(quadric)
    
    def setLength(self, newLength):
        self.length = newLength

    def getLength(self):
        return self.length

    def getBaseRadius(self):
        return self.base
    
    def getTopRadius(self):
        return self.top
