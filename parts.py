from OpenGL.GLU import * 
from OpenGL.GL import *
import imgui
from shapes import Cylinder
from utils import Axes

class Member():
    def __init__(self, length=1.0):
        self.cylinder = Cylinder(length=length, base=0.25, top=0.25)
        self.axis = Axes()
        self.length=length

    def draw(self):
        glPushMatrix()
        glColor3f(0.8, 0.8, 0.8)
        self.cylinder.draw()
        glTranslatef(0, 0, self.length // 2)
        self.axis.draw()
        glPopMatrix()

class Joint():
    def __init__(self, startAngle=0.0, name='Joint'):
        self.body = Cylinder(base=0.5, top=0.5, length=1)
        self.axis = Axes()
        self.angle = startAngle
        self.changed = False
        self.name = name

    def draw(self):
        self.changed, self.angle = imgui.slider_float(self.name, self.angle, -180.0, 180.0)
        glPushMatrix()
        glRotatef(self.angle, 0, 0, 1)
        self.axis.draw()
        glColor3f(0.5, 0, 0.5)
        self.body.draw()
        glPopMatrix()
    
    def getCurrentAngle(self):
        return self.angle
    

class Leg():
    def __init__(self, name: str):
        self.name = name
        self.coxa = JointMemberPair(name='Coxa')
        self.femur = JointMemberPair(name='Femur')
        self.tibia = JointMemberPair(name='Tiba')
    
    def draw(self):
        glPushMatrix()

        # Coxa
        self.coxa.draw()
        glRotatef(self.coxa.getCurrentAngle(), 0, 0, 1)
        glTranslatef(3.0, 0.0, 0.0)  # move along leg length

        # # Femur
        self.femur.draw()
        glRotatef(self.femur.getCurrentAngle(), 0, 0, 1)
        glTranslatef(3.0, 0.0, 0.0)

        # # Tibia
        # glRotatef(self.tibia.getCurrentAngle(), 0, 0, 1)
        # self.tibia.draw()

        glPopMatrix()


class JointMemberPair():
    def __init__(self, startAngle=0.0, name='Joint and Member'):
        # self.joint = Cylinder(base=0.5, top=0.5, length=1)
        self.joint = Joint()
        self.leg = Member(length=3.0)
        self.axis = Axes()
        self.angle = startAngle
        self.changed = False
        self.name = name

    def draw(self):
        # self.changed, self.angle = imgui.slider_float(self.name, self.angle, -180.0, 180.0)

        glPushMatrix()
        # glRotatef(self.joint.getCurrentAngle(), 0, 0, 1)
        self.joint.draw()
        glRotatef(self.joint.getCurrentAngle(), 0, 0, 1)
        self.angle = self.joint.getCurrentAngle()

        glColor3f(0.5, 0.5, 0.5)
        glTranslatef(0.0, 0.0, 0.5)
        glRotatef(90.0, 0, 1, 0)
        self.leg.draw()
        glPopMatrix()
    
    def getLength(self):
        return float(self.leg.length)

    def getCurrentAngle(self):
        # print(f'current angle {self.angle}')
        return float(self.angle)
