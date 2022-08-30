import gym
from gym import spaces
import numpy as np
import random

#MAX_REWARD = 1 # Maximum reward
cube_size = 10 # used to define boundaries for scale values
steps = 100000

class TopoEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    
    def __init__(self, constraint=1):
        super(TopoEnv, self).__init__()
        
        #self.reward_range = (0, MAX_REWARD)
        
        self.current_step = 0

        self.constraint = constraint # constraint is the minimum integral that should be achieved in the load-displacement curve

        self.last_performance = 0

        self.last_reward = 0
        
        # Define action space
        # Only one adjustment possible each step
        self.action_space = spaces.Discrete(12)
        # Multiple adjustments possible at the same time
        # self.action_space = spaces.MultiDiscrete([2,2,2,2,2,2])
        
        # Define observation space: [a,a,a,b,b,b] with a in [0.1,cube_size/5] and b in [0.0,359.9]
        self.observation_space = spaces.Box(low=np.array([0.1,0.1,0.1,0.0,0.0,0.0]), high=np.array([cube_size/2, cube_size/2, cube_size/2, 359.9, 359.9, 359.9]), dtype=np.float32)
        
        

    def step(self, action):
        # Execute one time step within the environment
        
        assert self.action_space.contains(action), "Invalid Action"

        # translate actions
        if action == 0:
            self.material.change(0,0)
        elif action == 1:
            self.material.change(0,1)
        elif action == 2:
            self.material.change(1,0)
        elif action == 3:
            self.material.change(1,1)
        elif action == 4:
            self.material.change(2,0)
        elif action == 5:
            self.material.change(2,1)
        elif action == 6:
            self.material.change(3,0)
        elif action == 7:
            self.material.change(3,1)
        elif action == 8:
            self.material.change(4,0)
        elif action == 9:
            self.material.change(4,1)
        elif action == 10:
            self.material.change(5,0)
        elif action == 11:
            self.material.change(5,1)

        new_value = max(self.material.get_material()) # placeholder calculation

        # TODO: reward calculation
        # evaluate material performance
        new_performance = self.performance(constraint=self.constraint, value=new_value)

        old_performance = self.last_performance

        if new_performance > old_performance:
            reward = 10*((new_performance/old_performance)-1) # alternative: 1.0
        else:
            reward = -1.0
        
        self.last_reward = reward
        self.last_performance = new_performance
        
        self.current_step += 1
        
        done = self.current_step >= steps
        
        observation = np.array(self.material.get_material(), dtype=np.float32) # get new material
        
        return observation, reward, done, {}
    

    def reset(self):
        # Reset the state of the environment to an initial state
        x = round(random.uniform(0.1, cube_size/2), 1)
        y = round(random.uniform(0.1,cube_size/2), 1)
        z = round(random.uniform(0.1,cube_size/2), 1)
        rot_x = round(random.uniform(0.0,359.9), 1)
        rot_y = round(random.uniform(0.0,359.9), 1)
        rot_z = round(random.uniform(0.0,359.9), 1)
        
        self.material = Material(x, y, z, rot_x, rot_y, rot_z)
        self.current_step = 0
        self.last_reward = 0

        current_value = 0.1 # dummy value
        performance = self.performance(constraint=self.constraint, value=current_value)
        self.last_performance = performance

        observation = np.array(self.material.get_material(), dtype=np.float32) # get new material
        
        return observation

    def render(self, mode='human', close=False):
        # Render the environment to the screen

        last_material = self.material.get_material()

        print(f'Step: {self.current_step}')
        print(f'Last material: {last_material}')
        print(f'Last reward: {self.last_reward}')

    def performance(self, constraint, value):
        # Calculate how "good" the material is compared proportionally to the constraint
        if value >= constraint:
            performance = value
        else:
            proportion = constraint / value
            factor = 1/(1+proportion)
            performance = value * factor

        return performance

    

class Material(object):
    
    def __init__(self, x, y, z, rot_x, rot_y, rot_z):
        self.state = [x, y, z, rot_x, rot_y, rot_z]
    
    def change(self, selection, action): # selection stands for 0..5th element, action for decrease (0) or increase (1)
        if action == 0:
            selected = self.state[selection]
            if selection < 3: # first three elements
                if selected == 0.1: # already at minimum
                    self.state[selection] = 0.1 #clamp
                else:
                    self.state[selection] -= 0.1
            else: # rotation selected
                if selected == 0.0: # already at minimum
                    self.state[selection] = 359.9 # rolling over
                else:
                    self.state[selection] -= 0.1
        
        if action == 1:
            selected = self.state[selection]
            if selection < 3: # first three elements
                if selected == cube_size/2: # already at maximum
                    self.state[selection] = cube_size/2 #clamp
                else:
                    self.state[selection] += 0.1
            else: # rotation selected
                if selected == 359.9: # already at maximum
                    self.state[selection] = 0.0 # rolling over
                else:
                    self.state[selection] += 0.1
    
    def get_material(self):
        return self.state
