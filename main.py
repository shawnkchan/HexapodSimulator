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
    glClearColor(0.09, 0.09, 0.09, 1)
    gluPerspective(45, display[0] / display[1], 0.1, 50.0)
    glTranslatef(0.0, 0.0, -20)

    # set the initial camera view
    glRotatef(-45, 1, 0, 0)
    glRotatef(-135, 0, 0, 1)
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

    # Run an infinite loop to render any new frames
    while running:
        clock.tick(60)
        
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == pygame.MOUSEWHEEL:
                if event.y > 0:
                    zoomVal += 0.3
                else:
                    zoomVal -= 0.3
            impl.process_event(event)

        impl.process_inputs()
        imgui.new_frame()
        # imgui.set_next_window_size(400, 200)
        # imgui.begin("Forward Kinematics Control Panel")

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

        # x, y = pygame.mouse.get_rel()
        # if pygame.mouse.get_pressed(num_buttons=3)[0]:
        #     xAngle -= y
        #     zAngle += x
        
        if pygame.mouse.get_pressed(num_buttons=3)[0] and pygame.key.get_pressed()[K_LCTRL]:
            pass
            # glTranslatef(0.5 * x, 0, 0.5 * y)

        # clean the canvas to refresh the frame
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        canvas.drawScene(xAngle, yAngle, zAngle, zoomVal) 

        # imgui.end()
        imgui.render()
        impl.render(imgui.get_draw_data())

        pygame.display.flip()

            
    pygame.quit()

if __name__ == "__main__":
    main()