from datetime import timedelta
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from canvas import Canvas
import imgui
from imgui.integrations.pygame import PygameRenderer


def main():
    # Initialize pygame
    pygame.init()
    display = (1000, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    glClearColor(1, 1, 1, 1)
    gluPerspective(45, display[0] / display[1], 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    # set the initial camera view
    glRotatef(-45, 1, 0, 0)
    glRotatef(-45, 0, 0, 1)
    glEnable(GL_DEPTH_TEST)
    clock = pygame.time.Clock()
    running = True

    # instantiate canvas object
    canvas = Canvas()
    xAngle, yAngle, zAngle, zoomVal = 0, 0, 0, 0

    # create imgui object
    imgui.create_context()
    impl = PygameRenderer()
    impl.io.display_size = display
    angle = 0.0


    # Run an infinite loop to render any new frames
    while running:
        timeDelta = clock.tick(60) / 1000.0
        
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            impl.process_event(event)

        impl.process_inputs()
        imgui.new_frame()
        imgui.set_next_window_size(300, 200)
        imgui.begin("Control Panel")
        changed, angle = imgui.slider_float("Rotation", angle, -180.0, 180.0)
        imgui.end()
        

        keys = pygame.key.get_pressed()
        if keys[K_LEFT]:
            zAngle += 1
        if keys[K_RIGHT]:
            zAngle -= 1
        if keys[K_UP]:
            xAngle += 1
        if keys[K_DOWN]:
            xAngle -= 1
        if keys[pygame.K_z]:
            zoomVal -= 0.1
        if keys[pygame.K_x]:
            zoomVal += 0.1

        # clean the canvas to refresh the frame
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        canvas.drawScene(xAngle, yAngle, zAngle, zoomVal)    

        imgui.render()
        impl.render(imgui.get_draw_data())

        pygame.display.flip()

            
    pygame.quit()

if __name__ == "__main__":
    main()