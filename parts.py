from gc import enable
from tracemalloc import start
from OpenGL.GLU import * 
from OpenGL.GL import *
import imgui
from shapes import Cylinder
from utils import Axes, TogglePanel
from kinematicsSolvers import dhTransformMatrix, endEffectorPosition, ikSolverLeg
import math
import numpy as np

class Member():
    def __init__(self, name='Member', length=1.0):
        self.cylinder = Cylinder(length=length, radius=0.25)
        self.axis = Axes()
        self._length = length
        self._name = name

    def draw(self):
        # self.changed, self.length = imgui.input_float(self.name, self.length, 0.1, 50)
        glPushMatrix()
        glColor3f(0.8, 0.8, 0.8)
        self.cylinder.length = self._length
        self.cylinder.draw()
        glTranslatef(0, 0, self._length / 2)
        self.axis.draw()
        glPopMatrix()

    @property
    def name(self):
        return self._name
    
    @property
    def length(self):
        return self._length
    
    @length.setter
    def length(self, value):
        self._length = value
    
    @property
    def radius(self):
        return self.cylinder.radius
    
    @radius.setter
    def radius(self, value):
        self.cylinder.radius = value


class Joint():
    def __init__(self, startAngle=0.0, minAngle=-180.0, maxAngle=180.0, name='Joint'):
        self.body = Cylinder(radius=0.5, length=1)
        self.axis = Axes()
        self._angle = startAngle
        self.changed = False
        self.name = name
        self._minAngle = minAngle
        self._maxAngle = maxAngle

    def draw(self):
        # imgui.slider_angle returns angles in radians
        glPushMatrix()
        glRotatef(self.angle, 0, 0, 1)
        self.axis.draw()
        glTranslatef(0, 0, -self.body.length / 2)
        glColor3f(0.5, 0, 0.5)
        self.body.draw()
        glPopMatrix()
    
    @property
    def angle(self):
        # TODO: get this to return in radians, getting confusing
        '''
        returns the joint's current yaw angle in degrees for easier use with glRotatef
        '''
        return math.degrees(self._angle)
    
    @angle.setter
    def angle(self, value):
        self._angle = value

    @property
    def radius(self):
        return self.body.radius
    
    @property
    def height(self):
        return self.body.length

    @property
    def minAngle(self):
        return self._minAngle
    
    @minAngle.setter
    def minAngle(self, value):
        self._minAngle = value

    @property
    def maxAngle(self):
        return self._maxAngle
    
    @maxAngle.setter
    def maxAngle(self, value):
        self._maxAngle = value
        

class JointMemberPair():
    # TODO: be nice if we can just inherit from Joint and Member classes
    def __init__(self, startAngle=0.0, name='JointMember', length=3.0):
        self.joint = Joint(name=f'{name} Joint', startAngle=startAngle)
        self.leg = Member(name=f'{name} Member', length=length)
        self.axis = Axes()
        self._angle = startAngle
        self.changed = False
        self._name = name

    def draw(self):
        glPushMatrix()
        self.joint.draw()
        glRotatef(self.joint.angle, 0, 0, 1)
        self._angle = self.joint.angle

        glColor3f(0.5, 0.5, 0.5)
        glRotatef(90.0, 0, 1, 0)
        self.leg.draw()
        glPopMatrix()
    
    @property
    def name(self):
        return self._name

    @property
    def length(self):
        return float(self.leg.length)
    
    @length.setter
    def length(self, value):
        # TODO: add joint width to the length in the future?
        self.leg.length = value

    @property
    def angle(self):
        '''
        Returns the related Joint object's angle
        '''
        return self._angle

    @angle.setter
    def angle(self, value):
        '''
        Sets the Joint object's angle
        '''
        self.joint.angle = value
    
    @property
    def minAngle(self):
        '''
        Minimum joint angle in radians
        '''
        return self.joint.minAngle
    
    @minAngle.setter
    def minAngle(self, value):
        self.joint.minAngle = value

    @property
    def maxAngle(self):
        '''
        Maximum joint angle in radians
        '''
        return self.joint.maxAngle
    
    @maxAngle.setter
    def maxAngle(self, value):
        self.joint.maxAngle = value

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
            coxa=self.coxa,
            femur=self.femur,
            tibia=self.tibia
            )

        self.reachablePositions = self.computeReachablePoints()

        # set the max and min angles for the femur and for the tibia
        maxFemurAngle = math.degrees((math.pi) - self.minimumLinkAngle(self.coxa, self.femur))
        minFemurAngle = -maxFemurAngle
        self.femur.maxAngle = maxFemurAngle
        self.femur.minAngle = minFemurAngle

        maxTibiaAngle = math.degrees((math.pi) - self.minimumLinkAngle(self.femur, self.tibia))
        minTibiaAngle = -maxTibiaAngle
        self.tibia.maxAngle = maxTibiaAngle
        self.tibia.minAngle = minTibiaAngle
    
    def minimumLinkAngle(self, link1: JointMemberPair, link2: JointMemberPair):
        '''
        Computes the minimum possible angle in radians between two JointMemberPairs.
        '''
        r1 = link1.joint.radius
        h1 = link1.leg.radius
        g1 = math.atan((h1 / 2) / r1)

        r2 = link2.joint.radius
        h2 = link2.leg.radius
        g2 = math.atan((h2 / 2) / r2)

        return g1 + g2
    
    def draw(self, togglePanel: TogglePanel):
        if togglePanel.updateReachablePointsClicked:
            self.reachablePositions = self.computeReachablePoints()

        # Draw all relevant control panels for Leg parameters
        if togglePanel.displayReachablePoints:
            self.drawReachablePoints()

        if togglePanel.isInStandingMode:
            self.standState()
        else:
            self.flattenState()

        self.drawControlPanel(enableInverseKinematics=togglePanel.enableInverseKinematics)

        glPushMatrix()
        # Draw IK's goal point if any
        self.ikSolver.draw()

        # Translate to the origin position
        glTranslatef(self.origin['x'], self.origin['y'], self.origin['z'])

        # Coxa
        self.coxa.draw()
        glRotatef(self.coxa.angle, 0, 0, 1)
        glRotatef(90.0, 1, 0, 0)
        coxaJointRadius = self.coxa.joint.radius
        coxaJointHeight = self.coxa.joint.height
        coxaLength = self.coxa.length
        femurXPosition = coxaLength
        glTranslatef(femurXPosition, 0.0, 0.0)  # move along leg length and center the Femur

        # Femur
        self.femur.draw()
        glRotatef(self.femur.angle, 0, 0, 1)
        femurLength = self.femur.length
        femurJointRadius = self.femur.joint.radius
        tibiaXPosition = femurLength
        glTranslatef(tibiaXPosition, 0.0, 0.0)

        # Tibia
        self.tibia.draw()
        glRotatef(self.tibia.angle, 0, 0, 1)
        glTranslatef(self.tibia.length, 0.0, 0.0)

        glPopMatrix()
    
    def drawControlPanel(self, enableInverseKinematics):
        imgui.set_next_window_size(400, 200)
        imgui.begin(f"{self.name} Control Panel")

        if enableInverseKinematics:
            self._inverseKinematicsControlPanel()
        else:
            self._forwardKinematicsControlPanel()
        
        self._linkLengthControlPanel()
        imgui.end()

    def _forwardKinematicsControlPanel(self):
        # Coxa controls
        _, coxaAngle = imgui.slider_angle(self.coxa.name, math.radians(self.coxa.angle), self.coxa.minAngle, self.coxa.maxAngle)
        self.coxa.angle = coxaAngle

        # Femur controls
        _, femurAngle = imgui.slider_angle(self.femur.name, math.radians(self.femur.angle), self.femur.minAngle, self.femur.maxAngle)
        self.femur.angle = femurAngle

        # Tibia controls
        _, tibiaAngle = imgui.slider_angle(self.tibia.name, math.radians(self.tibia.angle), self.tibia.minAngle, self.tibia.maxAngle)
        self.tibia.angle = tibiaAngle

    def _inverseKinematicsControlPanel(self):
        goalChanged, values = imgui.input_float3(f'{self.name} Goal Coordinates', self.ikSolver.xGoal, self.ikSolver.yGoal, self.ikSolver.zGoal)
        self.ikSolver.xGoal, self.ikSolver.yGoal, self.ikSolver.zGoal = values[0], values[1], values[2]

        self.coxa.angle = self.ikSolver.coxaAngle()
        self.femur.angle = self.ikSolver.femurAngle()
        self.tibia.angle = self.ikSolver.tibiaAngle()

    def _linkLengthControlPanel(self):
        coxaChanged, self.coxa.length = imgui.input_float(f'{self.coxa.name} length', self.coxa.length, 0.1, 50)
        femurChanged, self.femur.length = imgui.input_float(f'{self.femur.name} length', self.femur.length, 0.1, 50)
        tibiaChanged, self.tibia.length = imgui.input_float(f'{self.tibia.name} length', self.tibia.length, 0.1, 50)

    def computeReachablePoints(self):
        '''
        Draws all the end effector's reachable points
        '''
        reachablePositions = []
        finalMatrix = dhTransformMatrix(0, self.tibia.length, 0, 0) # coordinate frame at the end effector
        for i in np.arange(math.radians(-90), math.radians(90), 0.17):
            coxaTransformMatrix = dhTransformMatrix(0, 0, i, 0)
            for j in np.arange(math.radians(-90), math.radians(90), 0.17):
                femurTransformMatrix = dhTransformMatrix(math.pi / 2, self.coxa.length, j, 0)

                for k in np.arange(math.radians(-90), math.radians(90), 0.17):
                    tibiaTransformMatrix = dhTransformMatrix(0, self.femur.length, k, 0)

                    transformationMatrices =  [coxaTransformMatrix, femurTransformMatrix, tibiaTransformMatrix, finalMatrix]

                    endEffectorGlobalCoordinates = endEffectorPosition(0, 0, 0, transformationMatrices=transformationMatrices)

                    reachablePositions.append([endEffectorGlobalCoordinates[0], endEffectorGlobalCoordinates[1], endEffectorGlobalCoordinates[2]])
        return reachablePositions

    def drawReachablePoints(self):
        '''
        Draws the end effector's reachable points by varying the coxa angle, femur angle, and tibia angle from their minimum angle to maximum angle
        '''  
        glColor3f(0, 1, 1)
        glPointSize(3.0)
        glBegin(GL_POINTS)      
        for p in self.reachablePositions:
            glVertex3f(p[0], p[1], p[2])
        glEnd()
        
    def standState(self):
        '''
        Moves leg into standing orientation
        '''
        X_STANDING = 5.0
        Y_STANDING = 0.0
        Z_STANDING = -2.0

        self.ikSolver.setGoalCoordinates(X_STANDING, Y_STANDING, Z_STANDING)
    
    def flattenState(self):
        '''
        Moves leg into flatened orientation
        '''
        totalLength = self.coxa.length + self.femur.length + self.tibia.length
        self.ikSolver.setGoalCoordinates(totalLength, 0.0, 0.0)
