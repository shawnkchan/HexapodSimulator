import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import pygame

class Cylinder():
    def __init__(self, base=0.1, top=0.1, length=1.0, slices=32):
        self.base = base
        self.top = top
        self.length = length
        self.slices = slices

    def draw(self):
        quadric = gluNewQuadric()
        gluQuadricDrawStyle(quadric, GLU_FILL)
        gluCylinder(quadric, self.base, self.top, self.length, self.slices, 1)



class Axes():
    def __init__(self):
        self.unitCylinder = Cylinder()
    
    def draw(self):
        glPushMatrix()

        # z axis, red
        glColor3f(1, 0, 0)
        self.unitCylinder.draw()

        # x axis, blue
        glRotatef(90, 0, 1, 0)
        glColor3f(0, 0, 1)
        self.unitCylinder.draw()
    
        # y axis, green
        glRotatef(90, 1, 0, 0)
        glColor3f(0, 1, 0)
        self.unitCylinder.draw()

        glPopMatrix()


class Canvas():
    '''
    Representation of a drawing that gets rendered in a canvas
    '''
    def __init__(self):
        self.mainAxes = Axes()
    
    def drawScene(self, xAngle, yAngle, zAngle):
        glPushMatrix()

        # rotate around the scene
        glRotatef(zAngle, 0, 0, 1)
        glRotatef(xAngle, 1, 0, 0)
        glRotatef(yAngle, 0, 1, 0)

        # draw objects here
        self.mainAxes.draw()

        glPopMatrix()


def main():
    # Initialize pygame
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    glClearColor(1, 1, 1, 1)
    gluPerspective(45, display[0]/display[1], 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    # set the initial camera view
    glRotatef(-45, 1, 0, 0)
    glRotatef(-45, 0, 0, 1)
    glEnable(GL_DEPTH_TEST)
    clock = pygame.time.Clock()
    running = True

    # instantiate canvas object
    canvas = Canvas()
    xAngle, yAngle, zAngle = 0, 0, 0

    # Run an infinite loop to render any new frames
    while running:
        # clean the canvas to refresh the frame
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

        keys = pygame.key.get_pressed()
        if keys[K_LEFT]:
            zAngle += 1
        if keys[K_RIGHT]:
            zAngle -= 1
        if keys[K_UP]:
            xAngle += 1
        if keys[K_DOWN]:
            xAngle -= 1


        canvas.drawScene(xAngle, yAngle, zAngle)

        pygame.display.flip()
        clock.tick(60)
            
    pygame.quit()

if __name__ == "__main__":
    main()