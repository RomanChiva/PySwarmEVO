import numpy as np
import scipy as sp 
import simpy
import random 
import time
import pygame
from functions_sensors import  position_change, PID, process_sensor_data, angle_to_bearing, bearing_to_state, sensor_input
import pandas as pd




class drone(object):

    def __init__(self, env, initial_position,dist,screen,background, action_states):


        self.env = env
        self.dist = dist
        self.screen = screen
        self.background = background
        self.action_states = action_states
        

        #Regarding Position:
        self.position = initial_position
        self.target_position = initial_position
        self.neighborhood = None

        #Regarding actions:
        self.neighbors_static = None # Are any of my neighbors moving?
        self.stationary = True # Am I moving?
        self.currently_doing = 'still' # Current action
        self.threshold = 5 # Trigger probablility to take action
        self.k_move = 0.25 # PID proportionality gain
        self.k_adjust = 0.05 # Adjustment gain
        self.k_repel = 0.03 # Repulsion gain
        self.k_attract = 0.15 # Attraction gain
        self.t_adj = 3 # How many time steps you spend adjusting after you are done mving

        # Trigger simulation
        self.env.process(self.draw_drone())
        self.env.process(self.action_manager())
       
        



    # Function updates position of drone while moving:

    def move(self):

        self.position = PID(self.position,self.target_position, self.k_move)

                
            


    # Function updates position of drone while still: 

    def still(self):
        
        pass




    # Function updaes position of drone while adjusting

    def adjust(self):

        v = np.array([0,0])

        relative_positions = self.neighborhood - np.array(self.position)
        distances = np.array([np.linalg.norm(i) for i in relative_positions])
        
        
        # Attraction

        if np.all(distances > self.dist*np.sqrt(2)*1.01):
    
            for x, pos in enumerate(relative_positions):

                unit_vector = pos/distances[x]
                
                speed = self.k_attract*(distances[x]- self.dist*np.sqrt(2))**2
                vector = unit_vector*speed
                v = v + vector
        

        # Repulsion

        if np.any(distances < self.dist*0.97):

            direction = relative_positions[np.where(distances == np.amin(distances))][0]
            dist = np.amin(distances)
            unit_vector = direction/dist
    
            speed = self.k_repel*(np.amin(distances)- self.dist)**2

            vector = -1*unit_vector*speed
            v = v + vector


         # Adjustment   
        
        else:
            
            for x,pos in enumerate(relative_positions):

                angle = np.arctan2(pos[1], pos[0])*(180/np.pi)
                bearing = angle_to_bearing(angle)
                state = bearing_to_state(bearing)
                change = np.array(position_change(state, self.dist))
                target_position = np.array(self.neighborhood[x]) - change

                vector = target_position - np.array(self.position)
                velocity = vector*self.k_adjust
                v = v + velocity
        
        self.position = self.position + v





    # Controller of drone. Recieves sensor input and decides what action to take

    def action_manager(self):

        while True:
            #print(self.neighborhood)
            sensor_input(self.dist)
            if self.currently_doing == 'still':

                if self.neighbors_static.all():
                    self.currently_doing = 'adjust'
                    self.adjust()

                else:
                    self.still()
            
            elif self.currently_doing == 'adjust':
                
                target_state = process_sensor_data(self.position, self.neighborhood, self.action_states)
            
                if self.neighbors_static.all() and random.randint(0,100) < self.threshold and target_state >=0:
                    self.target_position = np.array(self.position) + np.array(position_change(target_state, self.dist))
                    self.stationary = False
                    self.currently_doing = 'move'
                    self.move()
                    
                elif self.neighbors_static.all() != True:
                    self.currently_doing = 'still'
                    self.stationary = True
                    self.still()
                else:
                    self.adjust()
                    
                    
            elif self.currently_doing == 'move':
                
                if np.linalg.norm(np.array(self.target_position) -np.array(self.position)) < 5:
                    self.position = self.target_position
                    self.currently_doing = 'adjust'
                    self.stationary = True
                    
                    # Mandatory Adjust for a while after you are done moving

                    for x in range(self.t_adj):

                        if self.neighbors_static.all() != True:
                            self.stationary = True
                            self.currently_doing = 'still'
                            self.still()
                            break

                        self.adjust()

                        yield self.env.timeout(1)


                elif self.neighbors_static.all() != True:
                    self.stationary = True
                    self.currently_doing = 'still'
                    self.still()
                else:
                    self.move()

            yield self.env.timeout(1)


    

    # Update Position of drone on screen

    def draw_drone (self):

        while True:
            
            draw_pos = [int(i) for i in self.position]

            if self.currently_doing == 'move':

                pygame.draw.circle(self.background, (81, 189 ,26 ,1), draw_pos, 10)

            elif self.currently_doing == 'adjust':

                pygame.draw.circle(self.background, (239, 163 ,64 ,1), draw_pos, 10)

            elif self.currently_doing == 'still':

                pygame.draw.circle(self.background, (212, 1 ,6 ,1), draw_pos, 10)


            yield self.env.timeout(1)

            
 



       


    

