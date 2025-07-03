from tracemalloc import start
from OpenGL.GLU import * 
from OpenGL.GL import *
import imgui
from shapes import Cylinder
from utils import Axes
from kinematicsSolvers import dhTransformMatrix, endEffectorPosition, ikSolverLeg
import math

class Member():
    def __init__(self, name='Member', length=1.0):
        self.cylinder = Cylinder(length=length, radius=0.25)
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
    
    def getRadius(self):
        return self.cylinder.getRadius()


class Joint():
    def __init__(self, startAngle=0.0, minAngle=-180.0, maxAngle=180.0, name='Joint'):
        self.body = Cylinder(radius=0.5, length=1)
        self.axis = Axes()
        self.angle = startAngle
        self.changed = False
        self.name = name
        self.minAngle = minAngle
        self.maxAngle = maxAngle

    def draw(self):
        # self.changed, self.angle = imgui.slider_angle(self.name, self.angle, self.minAngle, self.maxAngle) # imgui.slider_angle returns angles in radians
        glPushMatrix()
        glRotatef(self.getCurrentAngle(), 0, 0, 1)
        self.axis.draw()
        glColor3f(0.5, 0, 0.5)
        self.body.draw()
        glPopMatrix()
    
    def getCurrentAngle(self):
        '''
        returns the joint's current yaw angle in degrees for easier use with glRotatef
        '''
        return math.degrees(self.angle)

    def getRadius(self):
        return self.body.getRadius()
    
    def getJointHeight(self):
        return self.body.getLength()
    
    def setAngle(self, angle):
        if angle < self.minAngle:
            print('Desired angle is smaller than minimum possible angle.')
        elif angle > self.maxAngle:
            print('Desired angle is greater than maximum possible angle')
        else:
            self.angle = angle

    def setMinimumAngle(self, angle):
        self.minAngle = angle
    
    def setMaximumAngle(self, angle):
        self.maxAngle = angle
        

class JointMemberPair():
    # TODO: be nice if we can just inherit from Joint and Member classes
    def __init__(self, startAngle=0.0, name='JointMember', length=3.0):
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
        glTranslatef(0.0, 0.0, self.joint.getJointHeight() / 2)
        glRotatef(90.0, 0, 1, 0)
        self.leg.draw()
        glPopMatrix()
    
    def getLength(self):
        return float(self.leg.length)

    def getCurrentAngle(self):
        return float(self.angle)
    
    def setJointAngle(self, angle):
        self.joint.setAngle(angle)

    def setMinimumJointAngle(self, angle):
        self.joint.setMinimumAngle(angle)
    
    def setMaximumJoinAngle(self, angle):
        self.joint.setMaximumAngle(angle)

    def getMinimumJointAngle(self):
        return self.joint.minAngle
    
    def getMaximumJointAngle(self):
        return self.joint.maxAngle

class Leg():
    def __init__(self, name: str, coxaLength=3.0, femurLength=3.0, tibiaLength=3.0, xOrigin=0, yOrigin=0, zOrigin=0):


        self.name = name
        self.coxa = JointMemberPair(name='Coxa', length=coxaLength)
        self.femur = JointMemberPair(name='Femur', length=femurLength)
        self.tibia = JointMemberPair(name='Tibia', length=tibiaLength)
        self.origin = {'x': xOrigin, 'y': yOrigin, 'z': zOrigin}
        self.ikSolver = ikSolverLeg(
            origin=self.origin, 
            name=self.name,
            coxaLength=self.coxa.getLength(),
            femurLength=self.femur.getLength(),
            tibiaLength=self.tibia.getLength()
            )

        # initialise the DH transformation matrices for a point on the end-effector
        self.transformationMatrices = [
            dhTransformMatrix(math.pi / 2, self.coxa.getLength(), self.coxa.getCurrentAngle(), 0),
            dhTransformMatrix(0, self.femur.getLength(), self.femur.getCurrentAngle(), 0),
            dhTransformMatrix(0, self.tibia.getLength(), self.tibia.getCurrentAngle(), 0)
        ]

        # print(math.degrees(self.ikSolver.coxaAngle()))
        # print(math.degrees(self.ikSolver.femurAngle(self.femur.getLength(), self.tibia.getLength(), self.coxa.getLength())))
        # print(math.degrees(self.ikSolver.tibiaAngle(self.femur.getLength(), self.tibia.getLength(), self.coxa.getLength())))

        # set the max and min angles for the femur and for the tibia
        maxFemurAngle = math.degrees((math.pi) - self.minimumLinkAngle(self.coxa, self.femur))
        minFemurAngle = -maxFemurAngle
        self.femur.setMaximumJoinAngle(maxFemurAngle)
        self.femur.setMinimumJointAngle(minFemurAngle)

        maxTibiaAngle = math.degrees((math.pi) - self.minimumLinkAngle(self.femur, self.tibia))
        minTibiaAngle = -maxTibiaAngle
        self.tibia.setMaximumJoinAngle(maxTibiaAngle)
        self.tibia.setMinimumJointAngle(minTibiaAngle)

    
    def minimumLinkAngle(self, link1: JointMemberPair, link2: JointMemberPair):
        '''
        Computes the minimum possible angle in radians between two JointMemberPairs.
        '''
        r1 = link1.joint.getRadius()
        h1 = link1.leg.getRadius()
        g1 = math.atan((h1 / 2) / r1)

        r2 = link2.joint.getRadius()
        h2 = link2.joint.getRadius()
        g2 = math.atan((h2 / 2) / r2)

        return g1 + g2

    
    def draw(self):
        self.drawForwardKinematicsControlPanel()

        glPushMatrix()
        # Draw IK's goal point if any
        self.ikSolver.draw()

        # Translate to the origin position
        glTranslatef(self.origin['x'], self.origin['y'], self.origin['z'])

        # Coxa
        coxaAngle = self.ikSolver.coxaAngle()
        print('Coxa ', coxaAngle)
        self.coxa.setJointAngle(coxaAngle)
        self.coxa.draw()
        glRotatef(self.coxa.getCurrentAngle(), 0, 0, 1)
        glRotatef(90.0, 1, 0, 0)
        coxaJointRadius = self.coxa.joint.getRadius()
        coxaJointHeight = self.coxa.joint.getJointHeight()
        coxaLength = self.coxa.getLength()
        femurXPosition = coxaLength
        glTranslatef(femurXPosition, coxaJointRadius, -coxaJointHeight / 2)  # move along leg length and center the Femur

        # Femur
        femurAngle = self.ikSolver.femurAngle()
        print('Femur ', femurAngle)
        self.femur.setJointAngle(femurAngle)
        self.femur.draw()
        glRotatef(self.femur.getCurrentAngle(), 0, 0, 1)
        femurLength = self.femur.getLength()
        femurJointRadius = self.femur.joint.getRadius()
        tibiaXPosition = femurLength
        glTranslatef(tibiaXPosition, 0.0, 0.0)

        # Tibia
        tibiaAngle = self.ikSolver.tibiaAngle()
        print('Tibia', tibiaAngle)
        self.tibia.setJointAngle(tibiaAngle)
        self.tibia.draw()
        glRotatef(self.tibia.getCurrentAngle(), 0, 0, 1)
        glTranslatef(self.tibia.getLength(), 0.0, 0.0)

        glPopMatrix()

    def drawForwardKinematicsControlPanel(self):
        imgui.set_next_window_size(400, 200)
        imgui.begin("Forward Kinematics Control Panel")

        # Coxa controls
        _, coxaAngle = imgui.slider_angle(self.coxa.name, math.radians(self.coxa.getCurrentAngle()), self.coxa.getMinimumJointAngle(), self.coxa.getMaximumJointAngle())
        self.coxa.setJointAngle(coxaAngle)

        # Femur controls
        _, femurAngle = imgui.slider_angle(self.femur.name, math.radians(self.femur.getCurrentAngle()), self.femur.getMinimumJointAngle(), self.femur.getMaximumJointAngle())
        self.femur.setJointAngle(femurAngle)

        # Tibia controls
        _, tibiaAngle = imgui.slider_angle(self.tibia.name, math.radians(self.tibia.getCurrentAngle()), self.tibia.getMinimumJointAngle(), self.tibia.getMaximumJointAngle())
        self.tibia.setJointAngle(tibiaAngle)

        imgui.end()

    def setCoxaAngle(self, angle):
        self.coxa.setJointAngle(angle)
    
    def setFemurAngle(self, angle):
        self.femur.setJointAngle(angle)

    def setTibiaAngle(self, angle):
        self.tibia.setJointAngle(angle)
