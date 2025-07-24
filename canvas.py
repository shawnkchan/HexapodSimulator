from OpenGL.GL import *
from OpenGL.GLU import *
from utils import Axes, TogglePanel
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
        self.togglePanel = TogglePanel()
        
        
    
    def drawScene(self, xAngle, yAngle, zAngle, zoomVal):
        glPushMatrix()

        # rotate around the scene
        glRotatef(zAngle, 0, 0, 1)
        glRotatef(xAngle, 1, 0, 0)
        glRotatef(yAngle, 0, 1, 0)
        glTranslatef(zoomVal, zoomVal, zoomVal)

        if not self.hideAxes:
            self.mainAxes.draw()

        # Draw toggle panel
        self.togglePanel.draw()

        # Draw all objects
        for obj in self.objects:
            obj.draw(
                isInverseKinematicsEnabled=self.togglePanel.enableInverseKinematics,
                isDisplayReachablePoints=self.togglePanel.displayReachablePoints,
                updateReachablePointsClicked=self.togglePanel.updateReachablePointsClicked)

        glPopMatrix()

    