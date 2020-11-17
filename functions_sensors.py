import pygame
import time
import numpy as np
from scipy.spatial import distance_matrix
import itertools
import pygame
import pandas as pd



#----------------------------------------FUNCTIONS-------------------------------------






#Generates initial conditions around a central drone 
def initial_position_generator(screen, n, dist):

    positions = [[int(x/2) for x in screen.get_size()]]

    for x in range(n-1):

        switch = True

        position = [int(x/2) for x in screen.get_size()]

        while switch:

            position[0] += dist * np.random.randint(-1,2)
            position[1] += dist * np.random.randint(-1,2)

            if (position in positions):
                pass
            else:
                positions.append(position)
                switch = False


    return positions






 #Draws grid on background of pygame screen
def draw_grid(screen, background):

    for x in np.linspace(0,screen.get_size()[0],50):
        pygame.draw.line(background, (200,200,200), [x,0], [x,screen.get_size()[1] ], 1)

    for y in np.linspace(0,screen.get_size()[1],35):
        pygame.draw.line(background, (200,200,200), [0,y], [screen.get_size()[0],y ], 1)







#Process to to update screen

def update_screen(env, screen, background):
    while True:

        screen.blit(background, (0,0))
        background.fill((255,255,255))
        draw_grid(screen, background)
        pygame.display.flip()

        #time.sleep(0.5)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()


        yield env.timeout(1)
        



#Binary conversions



def bin_dec(state):

    return(int(''.join(map(lambda state: str(int(state)),state)),2))


def dec_bin(state):

    bin = '{0:08b}'.format(state)

    return [int(n) for n in bin]




#Read state action matrix file

def read_sam(path):

    df = pd.read_csv(path, header = None)
    df = df[0].str.split(' ', expand = True)
    df.set_index(0, inplace = True)

    return df


# Function to interpret neighbourhood data and decide what state it is best to move to

def process_sensor_data(position, neighborhood,action_states):

    try:
        relative = neighborhood - np.array(position)
        angles = np.arctan2(relative[:,1], relative[:,0])*(180/np.pi)
    except:
        return (-1)

    bearings = np.zeros(angles.shape)
    state = np.zeros(8)

    for i, angle in enumerate(angles):
        if angle >= 90:
            bearings[i] = 450 - angle
        else:
            bearings[i] = 90 - angle
    
    for angle in bearings:

        if angle >= 337.5 or angle <= 22.5:
            state[0] = 1
        else:
            state[int(np.round(angle/45))] = 1

    state = bin_dec(state) # Current state of the drone


    # Read target action from selected action state matrix
    try:
        movements = action_states.loc[str(state)].to_numpy()
        movements = movements[movements != np.array(None)]
        target_state = int(np.random.choice(movements,1))
    except:
        return (-1)
        
    
    return target_state # Returns target action




#Position change for a given action

def position_change(state, d):

        if state == 0:
            change = [0,d]
            
        elif state == 1:
            change = [d,d]

        elif state == 2:
            change = [d,0]

        elif state == 3:
            change = [d,-d]

        elif state == 4:
            change = [0,-d]

        elif state == 5:
            change = [-d,-d]

        elif state == 6:
            change = [-d,0]

        elif state == 7:
            change = [-d,d]


        return change


# Very basic PID controller
def PID (IP, target, k):

    
    
    input_x = IP[0]
    input_y = IP[1]

    target_x = target[0]
    target_y = target[1]

    chnage_x = (target_x-input_x)*k
    chnage_y = (target_y-input_y)*k

    output_x = input_x + chnage_x
    output_y = input_y + chnage_y


    return [output_x, output_y]




def euclidean_distance_matrix(drones):

    # Array with positions of drones
    positions = np.array([drone.position for drone in drones])
    #Set up euclidean distance matrix
    dist_mat = distance_matrix(positions, positions)

    return dist_mat, positions



# Transform angles to bearings
def angle_to_bearing(angle):

    if angle >= 90:
        bearing = 450 - angle
    else:
        bearing = 90 - angle

    return bearing

# Transform bearings to states
def bearing_to_state(bearing):

    if bearing >= 337.5 or bearing <= 22.5:
        state = 0
    else:
        state = int(np.round(bearing/45))
    
    return state



#-----------------------------------SENSORS------------------------------------------------

#Senses drones surroundigs and returns data on the surrounding neighbourhood



# Sensor is triggered at each time step for each drone

def sensor_input(dist):

    from drone_class import drone
    from main import drones as DRONES
    
    

    #Define euclidean distance matrix and a list containing the positions of all drones at each time step
    dist_mat, positions = euclidean_distance_matrix(DRONES)

    for i, vector in enumerate(dist_mat):
        
        neighborhood = [] # List containing positions of the drones in the neighborhood
        neighbors_static = [] # Boolean list indicating whether a drone is moving in the neighborhood

        for j, distance in enumerate(vector):
            # Leave a tolerance
            if distance > 0 and distance < dist*np.sqrt(2)*1.1:

                neighborhood.append(positions[j])
                neighbors_static.append(DRONES[j].stationary)

        #Feed input into the drones
        DRONES[i].neighborhood = np.array(neighborhood)
        DRONES[i].neighbors_static = np.array(neighbors_static)






#GRAVEYARD
            
'''


def motion_sensor(env, drones, dist):
    while True:
        
        #Refresh
        for x in drones:
            x.can_i_move = True

        #Define euclidean distance matrix from which to work and vector containing positions of all drones
        dist_mat, positions = euclidean_distance_matrix(drones)

        #Each row in the matrices contains distances from drone[N_row] to the others, Thus iterate over these

        for x, vector in enumerate(dist_mat):

            for y, dist in enumerate(vector):

                if dist > 1 and dist < (dist+10) and drones[y].stationary == False :
                    
                    drones[x].can_i_move = False
                    break

         
        yield env.timeout(1)





# I think there is no need to worry if it will adjust to a moving drone
#It only adjusts once its succesfully done moving. If a moving drone is
#encountered the process is interrupted



              

                
def closest_neighbor(env, drones, dist):

    while True:

        dist_mat, positions = euclidean_distance_matrix(drones)

        for counter, vector in enumerate(dist_mat):

            vector_sorted = sorted(vector)
            order_indices = [vector_sorted.index(i) for i in vector]
            for i in range(1,100):
                try:
                    drones[counter].closest_neighbor_position = positions[order_indices.index(1)]
                    break
                except:
                    pass

        yield env.timeout(1)



def state_sensor(env, drones,dist, action_states):
    while True:

        positions = [n.position for n in drones]

        #List storing states for all drones
        states = []

        #This loop finds local state for each drone
        for n in range(len(positions)):

            #Find which drones are close enough
            distance_vect = np.delete(positions, n, 0) - positions[n]
            distance_mag = np.linalg.norm(distance_vect, axis = 1)
            in_neighbourhood_bool = distance_mag <= dist*np.sqrt(2)+2
            #Add 2 above to avoid possible confusion

            in_neighbourhood = distance_vect[in_neighbourhood_bool]

            #Calculate the angles the drone makes with its surrouding drones
            angles = np.arctan2(in_neighbourhood[:,1], in_neighbourhood[:,0])*(180/np.pi)
            bearings = np.zeros(angles.shape)

            for n in range(len(angles)):

                if angles[n] >= 90:
                    bearings[n] = 450 - angles[n]
                else:
                    bearings[n] = 90 - angles[n]

            state = np.zeros(8)

            #From the angles extract the states
            for angle in bearings:

                if angle >= 337.5 or angle <= 22.5:
                    state[0] = 1
                else:
                    state[int(np.round(angle/45))] = 1


            state = bin_dec(state)

            states.append(state)

        #Now we already know the current state, so we randomly read off a neww state off te state action matrix

        for counter,state in enumerate(states):

            try:
                movements = action_states.loc[str(state)].to_numpy()
                movements = movements[movements != np.array(None)]
                selection = np.random.choice(movements,1)
                target_state = selection
            except:
                target_state = -1

            drones[counter].target_state = int(target_state)

        yield env.timeout(1)







def adjustments(env, drones, dist):

    while True:

        dist_mat, positions = euclidean_distance_matrix(drones)

        for counter, vector in enumerate(dist_mat):

            if drones[counter].adjust == True:

                #Find relative ordering and use it to find closest drone and their relative positions

                vector_sorted = sorted(vector)
                order_indices = [vector_sorted.index(i) for i in vector]
                pos1 = positions[counter]

                for i in np.arange(1,10,1):

                    try:
                        pos2 = positions[order_indices.index(i)]
                        break
                    except:
                        pass

                #Now find angle between points and use it to find relative state

                dist1_2 = np.array(pos1)-np.array(pos2)

                angle = np.arctan2(dist1_2[1], dist1_2[0])* 180 / np.pi
                print(angle)
                #Turn to bearing:

                if angle >= 90:
                    bearing = 450 - angle

                else:
                    bearing = 90 - angle
                
                #Define state

                if angle >= 337.5 or angle <= 22.5:
                    state = 1
                else:
                    state = int(np.round(angle/45))
                

                #Now caluclate correct position of drone
                
                correct_position = list(np.array(pos2) + np.array(position_change(state, dist)))

                drones[counter].position = correct_position

        yield env.timeout(1)






'''






'''



        #Initialise. Fresh at each time step

        for drone in drones:
            drone.neighbors = True
        
        idx_moving - [] # List containing indeices of moving drones

        for x, drone in enumerate(drones):

            if drone.stationary = False:

                idx_moving.append(x)

        #Now that we know which drones are moving, find drones in their neighbourhood and set neighbourhood to false

        position_list = np.array([x.position for x in drones])
    

        for index in idx_moving:

            distance_vectors = position_list - np.array(drones[index].position)

            for x, vector in distance_vectors:

                if np.linalg.norm(vector) < dist and np.linalg.norm(vector) > 1:

                    drones[x].neighbors = False


'''

















