import numpy as np
import scipy as sp 
import simpy
import random 
import time
from drone_class import drone 
from functions_sensors import initial_position_generator, update_screen, read_sam
import pygame



#PARAMETERS-----------------------------

N_drones = 4 # Number of drones in the simulation
separation = 30 # Separation between the drones
sim_time = 9999999999 # Length of simulation
pattern = 'triangle4' # Choose between: 'lineNE', 'triangle3', 'triangle4', 'triangle9'
time_speed = 0.05 # Length of simulation time step in seconds (Looks best at 0.05s/step)


#Display settings---------------------------------

(width, height) = (1500, 1050)
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("PySwarm")

background = pygame.Surface(screen.get_size())
background = background.convert()

# Simulation Settings ------------------------------------

path = 'state_action_matrices/state_action_matrix_{}.txt'.format(pattern)
action_states = read_sam(path) #Reads file with phat state action matrix

env = simpy.rt.RealtimeEnvironment(factor=time_speed)

initial_positions = initial_position_generator(screen, N_drones, separation)
drones = [drone(env, x, separation, screen,background, action_states) for x in initial_positions] # Declare drone objects


#RUN---------------------------------------------

env.process(update_screen(env, screen, background))

env.run(until=sim_time)



