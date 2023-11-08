import pygame
import os
import numpy as np

RED = (255, 0, 0)
GREY = (100,100,150)
BLUE = (100,60,255)
GREEN = (60,255,100)
PINK = (200,100,100)

FPS = 60   # frames per second

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800

def Rmat(degree):
    rad = np.deg2rad(degree) 
    c = np.cos(rad)
    s = np.sin(rad)
    R = np.array([ [c, -s, 0],
                   [s,  c, 0], [0,0,1]])
    return R

def Tmat(tx, ty):
    Translation = np.array( [
        [1, 0, tx],
        [0, 1, ty],
        [0, 0, 1]
    ])
    return Translation
def draw(screen, P, H, color=(100,200,200)):
    R = H[:2,:2]
    T = H[:2,2]
    Ptransformed = P @ R.T +T
    pygame.draw.polygon(screen, color = color, points=Ptransformed)
    return

        
def main():
    pygame.init() # initialize the engine

    pygame.display.set_caption("20221003 김시연")

    screen = pygame.display.set_mode( (WINDOW_WIDTH, WINDOW_HEIGHT))
    clock = pygame.time.Clock()
    
    w=150
    w1 =100
    h=40
    X = np.array([ [0,0],[w,0],[w,h],[0,h] ])
    G = np.array([[0,0],[w1,0],[w1,h],[0,h]])
    g = np. array([[0,0],[w/2,0],[w/2,h],[0,h]])
    position = [WINDOW_WIDTH/2,WINDOW_HEIGHT-100]
    jointangle1= 10
    jointangle2 = -30
    jointangle3 = 20

    grip_angle1 = 0
    grip_angle2 = 0

    keyboard_d1 = 0
    keyboard_d2 = 0
    keyboard_d3 = 0
    animation_active = False
    target_rotation_angle = 45

    basex=0
    movebasex = position[0]

    tick = 0

    done = False
    while not done:
        #  input handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        #keyboard inputs handling
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    keyboard_d3 = -3
                elif event.key == pygame.K_RIGHT:
                    keyboard_d3 = 3
                elif event.key == pygame.K_UP:
                    keyboard_d2 = -3
                elif event.key == pygame.K_DOWN:
                    keyboard_d2 = 3
                elif event.key == pygame.K_a:
                    basex = -1
                elif event.key == pygame.K_d:
                    basex =1
                elif event.key == pygame.K_w:
                    keyboard_d1 = -3
                elif event.key == pygame.K_s:
                    keyboard_d1 = 3
                elif event.key == pygame.K_SPACE:
                    if animation_active:
                        animation_active = False
                        grip_angle1 = 0
                        grip_angle2 = 0
                    else:
                        animation_active = True

            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    keyboard_d3 = 0
                elif event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                    keyboard_d2 = 0
                elif event.key == pygame.K_a or event.key == pygame.K_d:
                    basex=0
                elif event.key == pygame.K_w or event.key == pygame.K_s:
                    keyboard_d1 = 0

        jointangle2 += keyboard_d2
        jointangle1 += keyboard_d1
        jointangle3 += keyboard_d3
        movebasex += basex
        tick+=1   

        if animation_active:
            rotation_step = 1  # Adjust this value to control the speed of the animation
            if grip_angle1 > -target_rotation_angle:
                grip_angle1 -= rotation_step
            if grip_angle2 < target_rotation_angle:
                grip_angle2 += rotation_step
            if grip_angle1 == target_rotation_angle or grip_angle2 == -target_rotation_angle:
                rotation_step = 0
            if grip_angle1 <= 0 and grip_angle2 >= 0:
                rotation_step = 0

        # drawing
        screen.fill( (30, 40, 50))

        #base
        H0 = Tmat(movebasex,position[1]) @ Tmat(0, -h)
        x,y = movebasex, position[1]
        draw(screen, X, H0, color=(160,160,160))

        #wheels
        pygame.draw.circle(screen, GREY, (x,y),radius=30, width = 3)
        pygame.draw.circle(screen, GREY, (x,y), radius=10)
        pygame.draw.circle(screen, GREY, (x+150,y),radius=30, width = 3)
        pygame.draw.circle(screen, GREY, (x+150,y), radius=10)

        #arm1
        H01= H0@Tmat(w/2,0)  
        x, y = H01[0,2], H01[1,2] #joint position
        H1 = H01@Rmat(-90)@Tmat(0, -h/2)
        # draw(screen, X, H1, PINK) #arm1, 90 degree
        sway = 10*np.sin(np.deg2rad(tick))
        H11 = H1 @ Tmat(0,h/2) @Rmat(sway) @ Rmat(jointangle1)@ Tmat(0,-h/2)
        draw(screen, X, H11, color=(100,100,100))
        pygame.draw.circle(screen, GREY, (x,y), radius=20)

        #arm 2
        H02 = H11@Tmat(w,0) @Tmat(0,h/2) #joint 2 position
        x,y = H02[0,2], H02[1,2]
        H2= H02 @Rmat(jointangle2) @Tmat(0,-h/2)
        draw(screen, X, H2, color=(130,130,130))
        pygame.draw.circle(screen, GREY, (x,y), radius=20)

        #arm3
        H03 = H2@Tmat(w,0) @Tmat(0,h/2) #joint 3 position
        x,y = H03[0,2], H03[1,2]
        H3= H03 @Rmat(jointangle3) @Tmat(0,-h/2)
        draw(screen, X, H3, color=(150,150,150))
        pygame.draw.circle(screen, GREY, (x,y), radius=20)

        #gripper base
        H04 = H3@Tmat(w,0) @Tmat(0,h/2) #joint 3 position
        x,y = H04[0,2], H04[1,2]
        H4= H04 @Rmat(90) @Tmat(-w1/2,0) @Tmat(0,-h)
        draw(screen, G, H4, color=(150,150,150))
        pygame.draw.circle(screen, GREY, (x,y), radius=20)

        H05 = H4@Tmat(w1,0) @Tmat(0,h/2)
        H5= H05 @Rmat(-90) @Rmat(grip_angle1) @Tmat(0,-h/2)
        H06 = H4 @Tmat(0,h/2)
        H6 = H06 @Rmat(-90)@Rmat(grip_angle2) @Tmat(0,-h/2)
        draw(screen, g, H5, color=(130,130,130))
        draw(screen, g, H6, color=(130,130,130))

    
        # finish
        pygame.display.flip()
        clock.tick(FPS)
    # end of while
# end of main()

if __name__ == "__main__":
    main()