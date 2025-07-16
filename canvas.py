from OpenGL.GL import *
from OpenGL.GLU import *
from utils import Axes
from parts import Member, Joint, Leg
import imgui

class Canvas():
    '''
    Representation of the space in which objects gets rendered

    @param hideAxes whether to show or hide the global frame axes
    '''

    def __init__(self, hideAxes=False):
        self.mainAxes = Axes()
        self.hideAxes = hideAxes
        self.objects = [
            # Joint(name='Coxa Joint')
            Leg(name='Leg1', coxaLength=1.5, femurLength=3.0, tibiaLength=5.0, xOrigin=0.0, yOrigin=0.0, zOrigin=0.0)
            ]
        self.enableInverseKinematics = True
        self.displayReachablePoints = False
        
    
    def drawScene(self, xAngle, yAngle, zAngle, zoomVal):
        glPushMatrix()

        # rotate around the scene
        glRotatef(zAngle, 0, 0, 1)
        glRotatef(xAngle, 1, 0, 0)
        glRotatef(yAngle, 0, 1, 0)
        glTranslatef(zoomVal, zoomVal, zoomVal)

        self.isInverseKinematicsEnabled()
        self.isDisplayReachablePoints()

        # draw objects here
        if not self.hideAxes:
            self.mainAxes.draw()

        for j in self.objects:
            j.draw(isInverseKinematicsEnabled=self.enableInverseKinematics, isDisplayReachablePoints=self.displayReachablePoints)

        glPopMatrix()

    def isInverseKinematicsEnabled(self):
        imgui.set_next_window_size(200, 100)
        imgui.begin('Toggle Inverse Kinematics')
        _, self.enableInverseKinematics = imgui.checkbox('Enable Inverse Kinematics', self.enableInverseKinematics)
        imgui.end()
    
    def isDisplayReachablePoints(self):
        imgui.set_next_window_size(200, 100)
        imgui.begin('Show Reachable Points')
        _, self.displayReachablePoints = imgui.checkbox('Display Reachable Points', self.displayReachablePoints)
        imgui.end()