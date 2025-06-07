import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math

'''
positive rotation is clockwise looking from top-down view of the axis
'''


def draw(angle):
    glPushMatrix()
    glRotatef(angle, 0, 0, 1)
    glRotatef(xAngle, 1, 0, 0)
    
    # Draw link as a simple rectangle
    axes()

    glPopMatrix()

def cylinder(base=0.1, top=0.1, length=1.0):
    quadric = gluNewQuadric()
    gluQuadricDrawStyle(quadric, GLU_FILL)
    gluCylinder(quadric, base, top, length, 32, 1)

def axes():
    glPushMatrix()

    # z axis, red
    glColor3f(1, 0, 0)
    cylinder()

    # x axis, blue
    glRotatef(90, 0, 1, 0)
    glColor3f(0, 0, 1)
    cylinder()
  
    # y axis, green
    glRotatef(90, 1, 0, 0)
    glColor3f(0, 1, 0)
    cylinder()

    glPopMatrix()

    
    
# Initialize pygame
pygame.init()
display = (800, 600)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
glClearColor(1, 1, 1, 1)
gluPerspective(45, display[0]/display[1], 0.1, 50.0)
glTranslatef(0.0, 0.0, -5)
glRotatef(-45, 1, 0, 0)
glRotatef(-45, 0, 0, 1)
glEnable(GL_DEPTH_TEST)

angle = 0
xAngle = 0

# Main loop
clock = pygame.time.Clock()
running = True
while running:
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
    
    keys = pygame.key.get_pressed()
    if keys[K_LEFT]:
        angle += 1
    if keys[K_RIGHT]:
        angle -= 1
    if keys[K_UP]:
        xAngle += 1
        

    draw(angle)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
