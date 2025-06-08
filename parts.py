from OpenGL.GLU import * 
from OpenGL.GL import *
from shapes import Cylinder
from utils import Axes

class Member():
    def __init__(self, color, length):
        self.cylinder = Cylinder(length=length)
        self.color = color


    def draw(self):
        # self.color
        self.cylinder.draw()

class Joint():
    def __init__(self, angle):
        self.angle = angle
        self.body = Cylinder(base=0.5, top=0.5, length=1)
        self.axis = Axes()
    
    def draw(self):
        glPushMatrix()
        glRotatef(self.angle, 0, 0, 1)
        self.axis.draw()
        glColor3f(1, 0, 1)
        self.body.draw()
        glPopMatrix()
    

class Leg():
    pass

class Coxa():

    def draw(self):
        glPushMatrix()
