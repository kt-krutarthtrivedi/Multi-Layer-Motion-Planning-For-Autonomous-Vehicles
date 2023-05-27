import numpy as np
import math
import pygame
from pygame import K_RETURN, K_RIGHT, K_LEFT, K_UP, K_DOWN, KEYUP
from global_vars import*
from pygame.math import Vector2
from algorithm import euc_dist,local_planner
import numpy as np
from numpy import random


def drawGrid(WINDOW_WIDTH,WINDOW_HEIGHT,screen):
 #Set the size of the grid block
    field = np.zeros((WINDOW_WIDTH//blockSize,WINDOW_HEIGHT//blockSize))
    for x in range(0, WINDOW_WIDTH, blockSize):
        for y in range(0, WINDOW_HEIGHT, blockSize):
            rect = pygame.Rect(x, y, blockSize, blockSize)
            pygame.draw.rect(screen, BLACK, rect,1)
    return field

def draw_path(screen,path):
    for (i,j,k) in path:
        rect = pygame.Rect(i*5, j*5, 5, 5)
        pygame.draw.rect(screen, BLUE, rect)

def draw_PRM_path(screen,path,points):
    for (i,j) in path:
        pygame.draw.circle(screen,BLUE,(i,j),4)
    for p in points:
        p  = tuple(list(p))
        pygame.draw.circle(screen,BLACK,p,2)
    pygame.draw.lines(screen, GREEN,closed=False, points = path, width=1)


def find_nearest(burning,curr_node):
    min_dist = 1000000
    out = (0,0)
    for node in burning:
        for obs in node:
            goal = (obs[0]*3,obs[1]*3)
            d  = euc_dist(curr_node,goal)
            if d<min_dist:
                min_dist = d
                out = obs         
    return out



    

def generate_obstacles(screen,field):
    for (x, y), w in np.ndenumerate(field):
        color = GREEN if w == 1 else WHITE
        # size = np.sqrt(abs(w) / max_weight)
        size = blockSize
        rect = pygame.Rect(x*blockSize, y*blockSize, size, size)
        pygame.draw.rect(screen, color, rect)


def compute_static_obsmap(field):
    b = 0
    obs_map = []
    grid = WINDOW_WIDTH//blockSize
    T = grid*grid
    field = np.zeros((grid,grid))

    x_list = (5,5,5,5,15,15,15,15, 25,25,25,25)
    y_list = (10,15,20,30,10,15,20,30,10,15,20,30)


    for i in range(len(x_list)):
        field, obs = place_t5(field,x_list[i],y_list[i])
        obs_map.append(obs)
    

    return field,obs_map

def compute_static_obsmap_lane_cars(field):
    b = 0
    obs_map = []
    grid = WINDOW_WIDTH//blockSize
    T = grid*grid
    field = np.zeros((grid,grid))

    x_list = (6,10,14,18,22,26,30,34)
    y_list = (1,5,1,3,1,5,1,5)

    for i in range(len(x_list)):
        field, obs = place_t5(field,x_list[i],y_list[i])
        obs_map.append(obs)

    return field,obs_map

def compute_dynamic_obsmap_lane_cars(field):
    b = 0
    obs_map = []
    grid = WINDOW_WIDTH//blockSize
    T = grid*grid
    field = np.zeros((grid,grid))

    x_list = [10]
    y_list = [5]

    for i in range(len(x_list)):
        field, obs = place_t5(field,x_list[i],y_list[i])
        obs_map.append(obs)

    return field,obs_map


def compute_obsmap(field,p):
    b = 0
    obs_map = []
    grid = WINDOW_WIDTH//blockSize
    T = grid*grid
    field = np.zeros((grid,grid))
    
    while((b/T)<p):
        [r] = np.random.choice(np.arange(1,5),1)
        [x] = np.random.choice(np.arange(2,grid),1)
        [y]  = np.random.choice(np.arange(2,grid-3),1)
        if (y >=10 and y<=15) or (y >=20 and y<=25) or (y >=30 and y<=35):
            continue
        if(r==1):
            if(check_t1(field,x,y)):
                b+=4
                field,obs = place_t1(field,x,y)
                obs_map.append(obs)
            else:
                continue
        if(r==2):
            if(check_t2(field,x,y)):
                b+=4
                field,obs = place_t2(field,x,y)
                obs_map.append(obs)
            else:
                continue
        if(r==3):
            if(check_t3(field,x,y)):
                b+=4
                field,obs = place_t3(field,x,y)
                obs_map.append(obs)
            else:
                continue
        if(r==4):
            if(check_t4(field,x,y)):
                b+=4
                field,obs = place_t4(field,x,y)
                obs_map.append(obs)
            else:
                continue
    return field,obs_map


def check_t1(field,x,y):
        if(field[x][y]==0 and field[x][y+1]==0 and field[x][y+2]==0 and field[x][y+3]==0):
            return True
        else:
            return False
    
def check_t2(field,x,y):
    if(field[x][y]==0 and field[x][y+1]==0 and field[x][y+2]==0 and field[x-1][y+2]==0):
        return True
    else:
        return False

def check_t3(field,x,y):
        if(field[x][y]==0 and field[x][y+1]==0 and field[x-1][y+1]==0 and field[x-1][y+2]==0):
            return True
        else:
            return False

def check_t4(field,x,y):
        if(field[x][y]==0 and field[x][y+1]==0 and field[x][y+2]==0 and field[x-1][y+1]==0):
            return True
        else:
            return False

def check_t5(field,x,y):
        if(field[x][y]==0 and field[x+1][y]==0 and field[x+2][y]==0 and field[x+3][y]==0):
            return True
        else:
            return False

def place_t1(field,x,y):
    field[x][y] = 1
    field[x][y+1] = 1
    field[x][y+2] = 1
    field[x][y+3] = 1
    obs_index = [(x,y),(x,y+1),(x,y+2),(x,y+3)] 
    return field,obs_index

def place_t2(field,x,y):
    field[x][y] = 1
    field[x][y+1] = 1
    field[x][y+2] = 1
    field[x-1][y+2] = 1
    obs_index = [(x,y),(x,y+1),(x,y+2),(x-1,y+2)]
    return field,obs_index

def place_t3(field,x,y):
    field[x][y] = 1
    field[x][y+1] = 1
    field[x-1][y+1] = 1
    field[x-1][y+2] = 1
    obs_index = [(x,y),(x,y+1),(x-1,y+1),(x-1,y+2)]
    return field,obs_index

def place_t4(field,x,y):
    field[x][y] = 1
    field[x][y+1] = 1
    field[x][y+2] = 1
    field[x-1][y+1] = 1
    obs_index = [(x,y),(x,y+1),(x,y+2),(x-1,y+1)]
    return field,obs_index

def place_t5(field,x,y):            #For static cars
    field[x][y] = 1
    field[x+1][y] = 1
    field[x+2][y] = 1
    field[x+3][y] = 1
    obs_index = [(x,y),(x+1,y),(x+2,y),(x+3,y)] 
    return field,obs_index

class Player(pygame.sprite.Sprite):
    def __init__(self,pos=(80,40)):
        super(Player, self).__init__()
        self.width = 25
        self.height = 11
        self.surf = pygame.Surface((self.width, self.height))
        self.surf.fill((0, 0, 255))
        self.surf.set_colorkey(BLACK)  
        self.image = self.surf.copy()
        self.image.set_colorkey(BLACK)
        self.rect = self.image.get_rect()
        pygame.draw.circle(self.surf, RED, (self.rect.midright),4)
        self.rect.center = pos
        self.new_image = self.surf.copy()
        self.position = Vector2(pos)
        self.direction = Vector2(1, 0)  # A unit vector pointing rightward.
        self.speed = 0.01
        self.angle_speed = 0
        self.angle = 0
        self.i = 1
    
   
    def navigate(self,path,curr_node,t):
        blockSize = 5
        if t<len(path) and (euc_dist(path[t],path[-1])>2.5):
            ind = t
            # m = (path[ind][1]-path[ind-1][1])/(path[ind][0]-path[ind-1][0])
            # if ind == len(path)-1:
            #     ang = 0
            # else:
            ang = round(path[ind][2],2)
            # print("ang",ang)
            self.angle = round((ang*180/math.pi))
            # print(path[ind][2],self.angle)
            self.new_image = pygame.transform.rotate(self.surf, -self.angle)
            self.rect = self.new_image.get_rect()
            self.rect.centerx = (path[ind][0]*blockSize)
            self.rect.centery = (path[ind][1]*blockSize)
            reached = False
            curr_node = path[ind]
        else:
            reached = True
        return reached,curr_node

   
    def PRM_navigate(self,path,curr_node,index,obs_map,dynamic_obs):
        if index<len(path):
            # ang = math.atan2((path[ind][1]-path[ind-1][1]),(path[ind][0]-path[ind-1][0]))
            # print("curr_node",curr_node," ",index," ",path[index])
            state,reached_waypoint = local_planner(curr_node,path[index],obs_map,dynamic_obs)
            # print("new",state)
            if reached_waypoint:
                index+=1
            # print(path[ind][2],self.angle)
            self.angle = round((state[2]*180/math.pi))
            self.new_image = pygame.transform.rotate(self.surf, -self.angle)
            self.rect = self.new_image.get_rect()
            self.rect.centerx = state[0]
            self.rect.centery = state[1]
            reached = False
            curr_node = state
        else:
            reached = True
        return reached,curr_node,index



class Dynamic_obs(pygame.sprite.Sprite):
    def __init__(self,center):
        super(Dynamic_obs, self).__init__()
        self.width = 50
        self.height = 25
        self.surf = pygame.Surface((self.width, self.height))
        self.surf.fill(BLACK)
        self.rect = self.surf.get_rect(
            center=center
        )
        self.speed = 6
    


    # Move the sprite based on speed
    # Remove the sprite when it passes the left edge of the screen
    def update(self):
        self.rect.move_ip(self.speed, 0)
        if self.rect.right < 0:
            self.kill()

    def find_boundary_ego_vehicle(self):
        self.ego_vehicle = Player()
        ego_height_const = self.ego_vehicle.height/2
        ego_width_const = self.ego_vehicle.width/2
        return ego_width_const, ego_height_const

    def find_corners_obs(self):
        center_x = self.rect.center[0]
        center_y = self.rect.center[1]
        corner_x = []
        corner_y = []

        right_top_corner_x = center_x + self.width/2
        right_bottom_corner_x = center_x + self.width/2
        left_top_corner_x = center_x - self.width/2
        left_bottom_corner_x = center_x - self.width/2

        corner_x.append(right_top_corner_x)
        corner_x.append(right_bottom_corner_x)
        corner_x.append(left_bottom_corner_x)
        corner_x.append(left_top_corner_x)

        right_top_corner_y = center_y - self.height/2
        right_bottom_corner_y = center_y + self.height/2
        left_top_corner_y = center_y - self.height/2
        left_bottom_corner_y = center_y + self.height/2

        corner_y.append(right_top_corner_y)
        corner_y.append(right_bottom_corner_y)
        corner_y.append(left_bottom_corner_y)
        corner_y.append(left_top_corner_y)

        return corner_x, corner_y

    def get_boundary(self):
        corner_x,corner_y = self.find_corners_obs()
        width_const, height_const = self.find_boundary_ego_vehicle()
        bound_x = []
        bound_y = []
        width_const*=3
        height_const*=3
        bound_x_right_top = corner_x[0] + width_const
        bound_x.append(bound_x_right_top)
        bound_y_right_top = corner_y[0] - height_const
        bound_y.append(bound_y_right_top)
        bound_x_right_bottom= corner_x[1] + width_const
        bound_x.append(bound_x_right_bottom)
        bound_y_right_bottom = corner_y[1] + height_const
        bound_y.append(bound_y_right_bottom)
              
        bound_x_left_bottom = corner_x[2] - width_const
        bound_x.append(bound_x_left_bottom)
        bound_y_left_bottom = corner_y[2] + height_const
        bound_y.append(bound_y_left_bottom)
        bound_x_left_top = corner_x[3] - width_const
        bound_x.append(bound_x_left_top)
        bound_y_left_top = corner_y[3] - height_const
        bound_y.append(bound_y_left_top)

        return bound_x, bound_y
        