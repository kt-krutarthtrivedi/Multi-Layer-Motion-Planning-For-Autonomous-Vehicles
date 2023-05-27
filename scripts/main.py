from operator import index
from numpy import angle, append
import pygame
from env import Dynamic_obs, draw_PRM_path, drawGrid, generate_obstacles, compute_obsmap
from env import Player
from global_vars import*
from PRM import*
import matplotlib.pyplot as plt
import numpy as np
pygame.init()


# Fill the background with white
screen = pygame.display.set_mode([WINDOW_WIDTH,WINDOW_HEIGHT])

player = Player((15,15))

field = drawGrid(WINDOW_WIDTH,WINDOW_HEIGHT,screen)


running = True
count=0

field,obstacle_map = compute_obsmap(field,0.05)
initial_obs = len(obstacle_map)
obs_map = obstacle_map
generate_obstacles(screen,field)
# screen.fill(WHITE)

moving_obs = pygame.sprite.Group()
new_obs = Dynamic_obs(center=(60,200))
new_obs1 = Dynamic_obs(center=(20,360))
new_obs2 = Dynamic_obs(center=(40,520))
moving_obs.add(new_obs)
moving_obs.add(new_obs1)
moving_obs.add(new_obs2)

screen.blit(player.new_image,player.rect)
pygame.display.flip()

start = (1,1)
curr_node = (start[0],start[1],0)
goal = (550,550)
prm = PRMController(numOfRandomCoordinates=2000, allObs=obs_map, current=start,destination=goal,win_height=600,win_width=600)
path,points = prm.runPRM(initialRandomSeed=0,screen = screen)
# print("goal:",goal)

dynamic = []
ind = 1
draw_PRM_path(screen,path,points)
while running:
    count+=1    
    # screen1 = pygame.display.set_mode([WINDOW_WIDTH,WINDOW_HEIGHT])
    # print("see",burning_dict)
    screen.fill(WHITE)
    generate_obstacles(screen,field)
    draw_PRM_path(screen,path,points)

    if count%10==0:
        player.i+=1
        dynamic_obs_list = []
        for obs in moving_obs:
            Dynamic_obs.update(obs)
            bound_x, bound_y = Dynamic_obs.get_boundary(obs)
            dynamic_obs_list.append([bound_x,bound_y])
            # print(bound_x, bound_y)
            dynamic = dynamic_obs_list

    if count%2==0:
        reached,curr_node,ind = player.PRM_navigate(path,curr_node,index=ind,obs_map=obs_map,dynamic_obs=dynamic)
        # print(burning,extinguish)
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
  
    for obs in moving_obs:
        screen.blit(obs.surf,obs.rect)    

    screen.blit(player.new_image,player.rect)
    # Flip the display
    pygame.display.flip()



# Done! Time to quit.
pygame.quit()