from operator import index
from numpy import angle, append
import pygame
from env import *
from env import Player
from global_vars import*
from PRM import*
import matplotlib.pyplot as plt
import numpy as np

def spawn_lanes(screen):
    pygame.draw.line(screen,BLACK, (0, 5), (600, 5), width=2)
    pygame.draw.line(screen,BLACK, (0, 32), (600, 32), width=2)
    pygame.draw.line(screen,BLACK, (0, 68), (600, 68), width=2)
    pygame.draw.line(screen,BLACK, (0, 95), (600, 95), width=2)

prm_time = 0
pygame.init()

# Fill the background with white
screen = pygame.display.set_mode([WINDOW_WIDTH,WINDOW_HEIGHT])
pygame.display.set_caption('Static and Dynamic Obstacle Avoidance - Lane Maneuvering')

player = Player((15,15))

field = drawGrid(WINDOW_WIDTH,WINDOW_HEIGHT,screen)

running = True
count=0

field,obstacle_map = compute_static_obsmap_lane_cars(field)

initial_obs = len(obstacle_map)
obs_map = obstacle_map

generate_obstacles(screen,field)
spawn_lanes(screen)

screen.blit(player.new_image,player.rect)
pygame.display.flip()

#-------------------------- START and GOAL --------------------#
start = (1,1)
curr_node = (start[0],start[1],0)
goal = (590,15)

prm = PRMController(numOfRandomCoordinates=1500, allObs=obs_map, current=start,destination=goal,win_height=600,win_width=100)
path,points = prm.runPRM(initialRandomSeed=0,screen = screen)

dynamic = []
ind = 1
draw_PRM_path(screen,path,points)

while running:
    count+=1    
    screen.fill(WHITE)
    generate_obstacles(screen,field)
    spawn_lanes(screen)
    draw_PRM_path(screen,path,points)

    if count%10==0:
        reached,curr_node,ind = player.PRM_navigate(path,curr_node,index=ind,obs_map=obs_map,dynamic_obs=dynamic)
        if reached:
            running = False
    #
    # Did the user click the window close button?
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            # If the Esc key is pressed, then exit the main loop
            if event.key == pygame.K_ESCAPE:
                running = False
          
        # Check for QUIT event. If QUIT, then set running to false.
  
    screen.blit(player.new_image,player.rect)
    # Flip the display
    pygame.display.flip()



# Done! Time to quit.
pygame.quit()