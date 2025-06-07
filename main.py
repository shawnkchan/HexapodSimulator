import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from canvas import Canvas


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