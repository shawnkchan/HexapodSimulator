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

class TogglePanel():
    '''
    Class to toggle between different view modes of hexapod legs
    '''
    def __init__(self):
        self._enableInverseKinematics = True
        self._displayReachablePoints = False
        self._updateReachablePointsClicked = False
    
    @property
    def enableInverseKinematics(self):
        return self._enableInverseKinematics

    @property
    def displayReachablePoints(self):
        return self._displayReachablePoints

    @property
    def updateReachablePointsClicked(self):
        return self._updateReachablePointsClicked
    
    def draw(self):
        self.drawInverseKinematicsToggle()
        self.drawDisplayReachablePointsToggle()
    
    def drawInverseKinematicsToggle(self):
        imgui.set_next_window_size(200, 100)
        imgui.begin('Toggle Inverse Kinematics')
        _, self._enableInverseKinematics = imgui.checkbox('Enable Inverse Kinematics', self.enableInverseKinematics)
        imgui.end()
    
    def drawDisplayReachablePointsToggle(self):
        imgui.set_next_window_size(200, 100)
        imgui.begin('Show Reachable Points')
        _, self._displayReachablePoints = imgui.checkbox('Display Reachable Points', self.displayReachablePoints)
        self._updateReachablePointsClicked = imgui.button('Update Reachable points')
        imgui.end()
