from OpenGL.GLU import * 
from OpenGL.GL import *
from shapes import Cylinder
from utils import Axes

class Joint():
    def __init__(self, color, length):
        self.cylinder = Cylinder(length=length)
        self.color = color

    def draw(self):
        self.color
        self.cylinder.draw()
