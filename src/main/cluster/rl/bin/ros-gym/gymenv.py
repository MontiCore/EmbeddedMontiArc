import gym
from gym import spaces
import numpy as np
import random

from dynawrapper import DynaWrapper

#MAX_REWARD = 1 # Maximum reward
cube_size = 10 # used to define boundaries for scale values
steps = 100000

class TopoEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    
    def __init__(self, constraint):
        super(TopoEnv, self).__init__()
        
        #self.reward_range = (0, MAX_REWARD)
        
        self.current_step = 0

        self.constraint = constraint # constraint is the minimum integral that should be achieved in the load-displacement curve, interval: [0.1,100]

        self.last_penalty = 0

        self.last_reward = 0
        
        # Define action space
        # Only one adjustment possible each step
        self.action_space = spaces.Discrete(12)
        # Multiple adjustments possible at the same time
        # self.action_space = spaces.MultiDiscrete([2,2,2,2,2,2])
        
        # Define observation space: [c,a,a,a,b,b,b] with c in [0.1,100], a in [0.1,cube_size/5] and b in [0.0,359.9] reprepresenting constraint, scale and rotation respectively
        self.observation_space = spaces.Box(low=np.array([0.1,0.1,0.1,0.1,0.0,0.0,0.0]), high=np.array([100, cube_size/2, cube_size/2, cube_size/2, 359.9, 359.9, 359.9]), dtype=np.float32)
        
        

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

        new_penalty = self.materialPenalty()

        old_penalty = self.last_penalty

        if new_penalty < old_penalty:
            reward = 10*((old_penalty/new_penalty)-1) # alternative: 1.0
        else:
            reward = -1.0
        
        self.last_reward = reward
        self.last_penalty = new_penalty
        
        self.current_step += 1
        
        done = self.current_step >= steps
        
        observation = np.concatenate((np.array(self.constraint), np.array(self.material.get_material(), dtype=np.float32)), axis=None) # get new material and current constraint
        
        return observation, reward, done, {}
    

    def reset(self):
        # Reset the state of the environment to an initial state
        constraint = round(random.uniform(0.1, 100), 1)
        x = round(random.uniform(0.1, cube_size/2), 1)
        y = round(random.uniform(0.1,cube_size/2), 1)
        z = round(random.uniform(0.1,cube_size/2), 1)
        rot_x = round(random.uniform(0.0,359.9), 1)
        rot_y = round(random.uniform(0.0,359.9), 1)
        rot_z = round(random.uniform(0.0,359.9), 1)

        self.constraint = constraint
        self.material = Material(x, y, z, rot_x, rot_y, rot_z)
        self.current_step = 0
        self.last_reward = 0

        self.dyna = DynaWrapper(self.material.get_material())

        penalty = self.materialPenalty()
        self.last_penalty = penalty

        observation = np.concatenate((np.array(self.constraint), np.array(self.material.get_material(), dtype=np.float32)), axis=None) # get new material and constraint
        
        return observation

    def render(self, mode='human', close=False):
        # Render the environment to the screen

        last_material = self.material.get_material()[1:]
        last_constraint = self.constraint

        print(f'Step: {self.current_step}')
        print(f'Last material: {last_material}')
        print(f'Last constraint: {last_constraint}')
        print(f'Last reward: {self.last_reward}')

    def materialPenalty(self):
        self.dyna.simulate()
        energy_absorption = self.dyna.integrate()
        peak = self.dyna.getPeak()
        final_level = self.dyna.getFinalLevel()
        scale = self.material.get_material()[:3]
        avg_scale = sum(scale) / 3

        if energy_absorption >= self.constraint:
            energy_absorption_penalty = 0
        else:
            energy_absorption_penalty = self.constraint - energy_absorption

        peak_penalty = peak - final_level

        return avg_scale * (1 + energy_absorption_penalty + peak_penalty)

    

class Material(object):
    
    def __init__(self, x, y, z, rot_x, rot_y, rot_z):
        self.state = [x, y, z, rot_x, rot_y, rot_z]
    
    def change(self, selection, action): # selection stands for 0..5th element, action for decrease (0) or increase (1)
        if action == 0:
            selected = self.state[selection]
            if selection < 3: # first three elements
                if selected <= 0.1: # already at minimum
                    self.state[selection] = 0.1 #clamp
                else:
                    self.state[selection] -= 0.1
            else: # rotation selected
                if selected <= 0.0: # already at minimum
                    self.state[selection] = 359.9 # rolling over
                else:
                    self.state[selection] -= 0.1
        
        if action == 1:
            selected = self.state[selection]
            if selection < 3: # first three elements
                if selected >= cube_size/2: # already at maximum
                    self.state[selection] = cube_size/2 #clamp
                else:
                    self.state[selection] += 0.1
            else: # rotation selected
                if selected >= 359.9: # already at maximum
                    self.state[selection] = 0.0 # rolling over
                else:
                    self.state[selection] += 0.1
    
    def get_material(self):
        return self.state
