from tracemalloc import start
from OpenGL.GLU import * 
from OpenGL.GL import *
import imgui
from shapes import Cylinder
from utils import Axes
from inverseKinematicsSolver import ikSolver

class Member():
    def __init__(self, name='Member', length=1.0):
        self.cylinder = Cylinder(length=length, base=0.25, top=0.25)
        self.axis = Axes()
        self.length = length
        self.name = name

    def draw(self):
        self.changed, self.length = imgui.input_float(self.name, self.length, 0.1, 50)
        glPushMatrix()
        glColor3f(0.8, 0.8, 0.8)
        self.cylinder.setLength(self.length)
        self.cylinder.draw()
        glTranslatef(0, 0, self.length / 2)
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

    def getJointRadius(self):
        return self.body.getBaseRadius()
    
    def getJointHeight(self):
        return self.body.getLength()
    
    def setAngle(self, angle):
        self.angle = angle
        

class JointMemberPair():
    def __init__(self, startAngle=0.0, name='JointMember', length=3.0):
        # self.joint = Cylinder(base=0.5, top=0.5, length=1)
        self.joint = Joint(name=f'{name} Joint', startAngle=startAngle)
        self.leg = Member(name=f'{name} Member', length=length)
        self.axis = Axes()
        self.angle = startAngle
        self.changed = False
        self.name = name

    def draw(self):
        glPushMatrix()
        self.joint.draw()
        glRotatef(self.joint.getCurrentAngle(), 0, 0, 1)
        self.angle = self.joint.getCurrentAngle()

        glColor3f(0.5, 0.5, 0.5)
        glTranslatef(self.joint.getJointRadius(), 0.0, self.joint.getJointHeight() / 2)
        glRotatef(90.0, 0, 1, 0)
        self.leg.draw()
        glPopMatrix()
    
    def getLength(self):
        return float(self.leg.length)

    def getCurrentAngle(self):
        return float(self.angle)
    
    def setJointAngle(self, angle):
        self.joint.setAngle(angle)

class Leg():
    def __init__(self, name: str, coxaLength=3.0, femurLength=3.0, tibiaLength=3.0, xOrigin=0, yOrigin=0, zOrigin=0):
        self.name = name
        self.coxa = JointMemberPair(name='Coxa', length=coxaLength)
        self.femur = JointMemberPair(name='Femur', length=femurLength)
        self.tibia = JointMemberPair(name='Tibia', length=tibiaLength)
        self.origin = {'x': xOrigin, 'y': yOrigin, 'z': zOrigin}
        self.ikSolver = ikSolver(origin=self.origin, name=self.name)
        self.ikSolver.setGoalCoordinates(2.0, 2.0, 2.0)
    
    def draw(self):
        glPushMatrix()
        # Draw IK's goal point if any
        self.ikSolver.draw()

        # Translate to the origin position
        glTranslatef(self.origin['x'], self.origin['y'], self.origin['z'])

        # Coxa
        self.coxa.draw()
        glRotatef(self.coxa.getCurrentAngle(), 0, 0, 1)
        glRotatef(90.0, 1, 0, 0)
        coxaJointRadius = self.coxa.joint.getJointRadius()
        coxaJointHeight = self.coxa.joint.getJointHeight()
        coxaLength = self.coxa.getLength()
        femurXPosition = coxaLength + coxaJointRadius * 2
        glTranslatef(femurXPosition, coxaJointRadius, -coxaJointHeight / 2)  # move along leg length and center the Femur

        # Femur
        self.femur.draw()
        glRotatef(self.femur.getCurrentAngle(), 0, 0, 1)
        femurLength = self.femur.getLength()
        femurJointRadius = self.femur.joint.getJointRadius()
        tibiaXPosition = femurLength + femurJointRadius * 2
        glTranslatef(tibiaXPosition, 0.0, 0.0)

        # Tibia
        self.tibia.draw()
        glRotatef(self.tibia.getCurrentAngle(), 0, 0, 1)
        glTranslatef(self.tibia.getLength(), 0.0, 0.0)

        glPopMatrix()

    def setCoxaAngle(self, angle):
        self.coxa.setJointAngle(angle)
    
    def setFemurAngle(self, angle):
        self.femur.setJointAngle(angle)

    def setTibiaAngle(self, angle):
        self.tibia.setJointAngle(angle)
