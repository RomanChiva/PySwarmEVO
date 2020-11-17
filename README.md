# PySwarmEVO

PySwarm simulates self-organizing pattern formation behaviour based on the Coppola et al.(2019) paper: https://repository.tudelft.nl/islandora/object/uuid%3Ae84a1f51-7d3f-4b59-b9ea-ab4923607cf3

***
## How to run:



- Clone the repository
- Install requirements: `pip3 install -r requirements.txt`
- Execute main.py: `python3 main.py`

***
## Edit parameters:

There are a few parameters you can edit, these are found in main.py. They are the following:
```
#PARAMETERS-----------------------------

N_drones = 4 # Number of drones in the simulation
separation = 30 # Separation between the drones
sim_time = 9999999999 # Length of simulation
pattern = 'triangle4' # Choose between: 'lineNE', 'triangle3', 'triangle4', 'triangle9'
time_speed = 0.05 # Length of simulation time step in seconds (Looks best at 0.05s/step)
```
***
## While Running:

While running, the simulation should look like this: 

![PySwarm](https://user-images.githubusercontent.com/57629929/99413282-e7716900-28f5-11eb-9661-7cd370ab87b2.png)

- **Green**: Moving
- **Orange**: Adjusting position
- **Red**: Still
