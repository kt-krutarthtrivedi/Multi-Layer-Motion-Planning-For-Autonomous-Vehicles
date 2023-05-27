from hashlib import new
from os import stat_result
import numpy as np
import math
from global_vars import*
from collections import deque
import heapq

blockSize = 5

grid_width = WINDOW_WIDTH//blockSize
grid_height = WINDOW_HEIGHT//blockSize


def new_field(field):
    new_field = np.zeros((grid_width,grid_height))
    for i in range(grid_width):
        for j in range(grid_height):
            new_field[i][j] = field[(i*5)//15][(j*5)//15]
    return new_field


def euc_dist(node,goal):
    # Compute Eucledian Distance
    h =pow((goal[1]-node[1]),2)+pow((goal[0]-node[0]),2)
    h =  round(math.sqrt(h),2)
    return h

def heuristic2(field):
    h2 = field*1000
    for i in range(field.shape[0]):
        for j in range(field.shape[1]):
            if field[i][j]==1:
                h2 = potfield(h2,i,j)
    return h2
                
def h3(node,goal):
    # Compute Eucledian Distance
    # print("hey",goal[2],node[2])
    t= node[2]
    if node[2]>(math.pi/2):
        t = node[2]-math.pi
    h = abs(goal[2]-t)
    # print("h3",h3)
    return h

def potfield(h2,x,y):
    di = [1,0,-1,2,-2]
    dj = [1,0,-1,2,-2]
    for i in di:
        for j in dj:
            rr = x+i
            rc = y+j
            if rr<0 or rc<0:
                continue
            elif rr>=grid_width or rc>=grid_height:
                continue
            elif i==0 and j==0:
                continue
            elif i==2 or i==-2 or j==-2 or j==2:
                h2[rr][rc] += 200
                continue
            else:
                h2[rr][rc] += 1000
    return h2


def dijkstra_path(prev,goal,start):
    i,j,k = int(goal[0]),int(goal[1]),goal[2]
    path = []
    path.append((i,j,prev[i][j][2]))
    while(True):
        # print(prev[i][j][2])
        path.append((prev[i][j][0],prev[i][j][1],prev[i][j][2]))
        if prev[i][j][0]==start[0] and prev[i][j][1]==start[1] and prev[i][j][2]==start[2]:
            break
        # else:
        #     print("nonono")
        m,n = int(i),int(j)
        i = int(prev[m][n][0])
        j = int(prev[m][n][1])
        
    return path

def Astar(field,start,goal):
    field = new_field(field)
    cost = np.ones((grid_width,grid_height))*float('inf')
    h2 = heuristic2(field)
    visited = np.zeros((grid_width,grid_height))
    u_s_val = [2, 0, -2]
    u_phi_val = [math.pi/6,-math.pi/6,0]
    L = 15
    dirn = 1
    prev = np.full((grid_width,grid_height,3),(0.0,0.0,0.0))
    path = deque()
    cost[start[0]][start[1]] = 0
    pq = [(0,(start[0],start[1]),start[2],dirn)]
    reverse_cost = 300
    forward_cost = 10
    while len(pq)>0:
        cost_list = []
        k1 = 0
        k2 = 10
        k3 = 0 
        k4 = 1
        curr_distance,curr_node,curr_head,curr_dirn = heapq.heappop(pq)
        visited[curr_node[0]][curr_node[1]] = 1
        if curr_node[0] == goal[0] and curr_node[1] ==goal[1] and curr_head==goal[2]:
            path = dijkstra_path(prev,goal,start)
            return path
        # elif euc_dist((curr_node[0],curr_node[1],curr_head),goal)<20:
        #     reverse_cost = 5
        #     forward_cost = 5
        #     k1 = 0
        #     k2 = 0
        #     k3 = 200
        for u_s in u_s_val:
            for u_phi in u_phi_val:
                theta_next = round((u_s * round(math.tan(u_phi),3) / L),3) + curr_head
                rr = round((u_s * math.cos(theta_next)) + curr_node[0])
                rc = round((u_s * math.sin(theta_next)) + curr_node[1])
                position = rr, rc
                heading = round(theta_next,2)
                # print("heading",theta_next)
                if rr<0 or rc<0:
                    continue
                elif rr>=grid_width or rc>=grid_height:
                    continue
                # elif field[rr][rc] == 1 or visited[rr][rc]:
                #     continue
                elif (u_s<0):
                    new_dirn = -1
                    dirn_cost = abs(curr_dirn-(new_dirn))
                    # print("dirn_cost",dirn_cost)
                    new_cost = cost[curr_node[0]][curr_node[1]]+reverse_cost+(k2*dirn_cost)+(euc_dist((rr,rc),goal)*(1+k1*abs(u_phi)))+(h2[rr][rc])+(k3*h3((rr,rc,heading),goal))
                    # *(1+(9/math.pi)*abs(u_phi))
                else:   
                    new_dirn = 1
                    dirn_cost = abs(curr_dirn-(new_dirn))
                    # print("dirn_cost",dirn_cost)
                    new_cost = cost[curr_node[0]][curr_node[1]]+forward_cost+(k2*dirn_cost)+(euc_dist((rr,rc),goal)*(1+k1*abs(u_phi)))+(h2[rr][rc])+(k3*h3((rr,rc,heading),goal)) 
                    # *(1+(9/math.pi)*abs(u_phi))
                if (new_cost<cost[rr][rc]):
                    cost[rr][rc] = new_cost
                    # print("look",curr_node[0],curr_node[1],curr_head)
                    cost_list.append((new_cost,position,heading))
                    prev[rr][rc] = [curr_node[0],curr_node[1],curr_head]
                    heapq.heappush(pq,(new_cost,position,heading,new_dirn))
        # print("dekh bc",cost_list)
    # print("out",field)
    path = dijkstra_path(prev,goal,start)
    return path


def check_dynamic_collision(dynamic_obs,state):
    x,y,theta = state
    for obs in dynamic_obs:
        if (x<=obs[0][0] and x>=obs[0][2]) and (y<=obs[1][1] and y>=obs[1][0]):
            return True
    return False

def check_static_collision(obs_map,state):
    x,y,theta = state
    p_x = x//15
    p_y = y//15
    for obs in obs_map:
        if(obs[0] == (p_x,p_y) or obs[1] == (p_x,p_y) or obs[2] == (p_x,p_y) or obs[3] == (p_x,p_y)):
            return True
    return False

def isfree(state,obs,dynamic_obs):
    x,y,theta = state
    px = x//15
    py = y//15
    if x<0 or y<0:
        return False
    elif x>=WINDOW_WIDTH or y>=WINDOW_HEIGHT:
        return False
    elif check_static_collision(obs,state):
        return False
    elif check_dynamic_collision(dynamic_obs,state):
        return False
    else:
        return True

def local_planner(curr_state,goal,obs_map,dynamic_obs):
    reached_goal = False
    L = 12
    curr_nodex,curr_nodey,curr_head = curr_state
    u_s_val = [3,0,-3]
    u_phi_val = [math.pi/3,-math.pi/3,0]
    states = []
    for u_s in u_s_val:
        for u_phi in u_phi_val:
            theta_next = round((u_s * round(math.tan(u_phi),3) / L),3) + curr_head
            rr = round((u_s * math.cos(theta_next)) + curr_nodex)
            rc = round((u_s * math.sin(theta_next)) + curr_nodey)
            heading = round(theta_next,2)
            new_state = rr,rc,heading
            if isfree(new_state,obs_map,dynamic_obs):
                states.append(new_state)
    # print(states)
    if len(states)==0:
        print("No Free state")
    min_dist = 10000
    for state in states:
        node = state[0],state[1]
        dist = euc_dist(node,goal)
        if dist<min_dist:
            out = state
            min_dist = dist
        # print("min",min_dist," ",out)
    if min_dist<=10:
        reached_goal = True
    return out,reached_goal

        



