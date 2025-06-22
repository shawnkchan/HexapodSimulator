from OpenGL.GL import *
from OpenGL.GLU import *

class Cylinder():
    def __init__(self, radius=0.1, length=1.0, slices=32):
        self.radius = radius
        self.length = length
        self.slices = slices

    def draw(self):
        quadric = gluNewQuadric()
        gluQuadricDrawStyle(quadric, GLU_FILL)
        gluCylinder(quadric, self.radius, self.radius, self.length, self.slices, 1)
        gluDeleteQuadric(quadric)
    
    def setLength(self, newLength):
        self.length = newLength

    def getLength(self):
        return self.length

    def getRadius(self):
        return self.radius

class Cone(Cylinder):
    def __init__(self, baseRadius=0.1, tipRadius=0.01, length=1.0, slices=32):
        self.baseRadius = baseRadius
        self.tipRadius = tipRadius
        self.length = length
        self.slices = slices
    
    def draw(self):
        quadric = gluNewQuadric()
        gluQuadricDrawStyle(quadric, GLU_FILL)
        gluCylinder(quadric, self.baseRadius, self.tipRadius, self.length, self.slices, 1)
        gluDeleteQuadric(quadric)
        