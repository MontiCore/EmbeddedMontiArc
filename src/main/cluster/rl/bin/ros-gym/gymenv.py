import os
import pathlib
import gym
from gym import spaces
import numpy as np
import random

from dynawrapper import DynaWrapper

cube_size = 100 # used to define boundaries for scale values (in mm)
MIN_SCALE = 10 # minimum scale for the structures
MIN_CONSTRAINT = 1 # minimum energy absorption constraint (in Jules)
MAX_CONSTRAINT = 100 # maximum energy absorption constraint (in Jules)
steps = 39 # this is the maximum number of steps, to reach any material
SCALE_STEP = 10 # step size for scale in mm (min. 1)
ROT_STEP = 10 # step size for rotation in degree (min. 1)

this_directory = pathlib.Path(__file__).parent.resolve()
project_root = os.path.join(this_directory, '..', '..', '..')

log_file = os.path.join(project_root, 'logs', 'env.log')
results = os.path.join(project_root, 'logs', 'results.log')
constraint_file = os.path.join(project_root, 'rl', 'constraint.txt')

class TopoEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    
    def __init__(self, constraint):
        super(TopoEnv, self).__init__()
        
        self.current_step = 0

        self.set_constraint = constraint # constraint is the minimum integral that should be achieved in the load-displacement curve, interval: [0.1,100]

        self.last_penalty = 0.0

        self.last_reward = 0.0
        
        # Define action space
        # Only one adjustment possible each step
        self.action_space = spaces.Discrete(12)
        # Multiple adjustments possible at the same time
        # self.action_space = spaces.MultiDiscrete([2,2,2,2,2,2])
        
        MAX_SCALE = int(cube_size / 2)
        # Define observation space: [c,a,a,a,b,b,b] with c in [MIN_CONSTRAINT,MAX_CONSTRAINT],
        # a in [MIN_SCALE,cube_size/2] and b in [0,179] reprepresenting constraint, scale and rotation respectively
        self.observation_space = spaces.Box(low=np.array([MIN_CONSTRAINT, MIN_SCALE, MIN_SCALE, MIN_SCALE, 0, 0, 0]), high=np.array(
            [MAX_CONSTRAINT, MAX_SCALE, MAX_SCALE, MAX_SCALE, 179, 179, 179]), dtype=np.int32)
        
        

    def step(self, action):
        logging = open(log_file, 'a')
        logging.write("STEP \n")
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

        self.dyna.updateMaterial(self.material.get_material())
        logging.write("Updated Material \n")
        new_penalty = self.materialPenalty()
        logging.write("Got penalty\n")
        old_penalty = self.last_penalty

        if new_penalty < old_penalty:
            reward = 10*((old_penalty/new_penalty)-1) # alternative: 1.0
        else:
            reward = -10.0

        logging.write("Got reward: " + str(reward) + "\n")

        self.last_reward = reward
        self.last_penalty = new_penalty
        
        logging.write("This was step " + str(self.current_step) + "\n")
        logging.close()

        self.current_step += 1
        
        done = self.current_step >= steps

        # get new material and current constraint
        observation = np.concatenate((np.array(self.constraint), np.array(self.material.get_material(), dtype=np.int32)), axis=None)
        
        return observation, reward, done, {}
    

    def reset(self):
        logging = open(log_file, 'a')
        logging.write("RESET \n")
        # Reset the state of the environment to an initial state
        if self.set_constraint:
            with open(constraint_file, 'r') as file:
                constraint = int(file.readline())
        else:
            constraint = int(round(random.uniform(MIN_CONSTRAINT, MAX_CONSTRAINT), 0))

        MAX_SCALE = int(cube_size / 2)
        scales = list(filter(lambda x: (x % SCALE_STEP == 0), range(MIN_SCALE, MAX_SCALE + 1)))
        rotations = list(filter(lambda x: (x % ROT_STEP == 0), range(180)))
        x = random.choice(scales)
        y = random.choice(scales)
        z = random.choice(scales)
        rot_x = random.choice(rotations)
        rot_y = random.choice(rotations)
        rot_z = random.choice(rotations)

        self.constraint = constraint
        self.material = Material(x, y, z, rot_x, rot_y, rot_z)
        self.current_step = 0
        self.last_reward = 0.0

        self.dyna = DynaWrapper(self.material.get_material())
        logging.write("Finished initializing DynaWrapper \n")
        logging.close()
        with open(results, 'a') as result_log:
            result_log.write("Constraint: " + str(self.constraint) + "\n")
            result_log.write("Structure init: " + str(self.material.get_material()) + "\n")
        self.last_penalty = 0.0

        observation = np.concatenate((np.array(self.constraint), np.array(self.material.get_material(), dtype=np.int32)), axis=None) # get new material and constraint
        
        return observation

    def render(self, mode='human', close=False):
        # Render the environment to the screen

        last_material = self.material.get_material()
        last_constraint = self.constraint

        print(f'Step: {self.current_step}')
        print(f'Last material: {last_material}')
        print(f'Last constraint: {last_constraint}')
        print(f'Last reward: {self.last_reward}')

    def materialPenalty(self):
        logging = open(log_file, 'a')
        logging.write("MATERIALPENALTY \n")
        logging.flush()
        self.dyna.simulate()
        logging.write("Dyna finished \n")
        energy_absorption = self.dyna.integrate()
        peak = self.dyna.getPeak()
        final_level = self.dyna.getFinalLevel()
        scale = self.material.get_material()[:3]
        avg_scale = sum(scale) / 3 # penalty for mass (approximated by the mean scale)

        if energy_absorption >= self.constraint: # penalty for not fulfilling energy absorption constraint
            energy_absorption_penalty = 0.0
        else:
            energy_absorption_penalty = self.constraint - energy_absorption

        peak_penalty = peak - final_level
        if peak_penalty <= 0.0:
            peak_penalty = 0.0

        logging.write("Finished calculating penalty \n")
        logging.close()
        penalty = (energy_absorption_penalty * (1 + peak_penalty + (int(cube_size / 2) - avg_scale)))
        # alternative function
        # penalty = (int(cube_size / 2) - avg_scale) * (1 + energy_absorption_penalty + peak_penalty)
        with open(results, 'a') as result_log:
            result_log.write("EA: " + str(energy_absorption) + "\n")
            result_log.write("PCF: " + str(peak) + "\n")
            result_log.write("Scale: " + str(avg_scale) + "\n")
            result_log.write("PENALTY: " + str(penalty) + "\n")
        return penalty

    

class Material(object):
    
    def __init__(self, x, y, z, rot_x, rot_y, rot_z):
        self.state = [x, y, z, rot_x, rot_y, rot_z]
    
    def change(self, selection, action): # selection stands for 0..5th element, action for decrease (0) or increase (1)
        if action == 0:
            selected = self.state[selection]
            if selection < 3: # first three elements
                new_scale = selected - SCALE_STEP
                if new_scale <= MIN_SCALE: # already at minimum
                    self.state[selection] = MIN_SCALE # clamp
                else:
                    self.state[selection] = new_scale
            else: # rotation selected
                if selected <= 0: # already at minimum
                    self.state[selection] = (180-ROT_STEP) # rolling over
                else:
                    self.state[selection] -= ROT_STEP
        
        if action == 1:
            selected = self.state[selection]
            if selection < 3: # first three elements
                new_scale = selected + SCALE_STEP
                if new_scale >= int(cube_size/2): # already at maximum
                    self.state[selection] = int(cube_size/2) # clamp
                else:
                    self.state[selection] = new_scale
            else: # rotation selected
                new_rotation = selected + ROT_STEP
                self.state[selection] = new_rotation % 180 # roll over if at maximum
    
    def get_material(self): # returns material corrected to the input of the network
        return self.state
