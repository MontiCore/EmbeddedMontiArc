# (c) https://github.com/MontiCore/monticore  
import gym
import rospy
import time
import _thread
import time
import numpy as np

from gym_torcs import TorcsEnv

from std_msgs.msg import Float32MultiArray, Bool, Int32, Float32
from gym import wrappers

NONE = 3
INFO = 2
DEBUG = 1


class RosTorcsConnector(object):
    state_topic = '/torcs/state'
    terminate_topic = '/torcs/terminal'
    step_topic = '/torcs/step'
    reset_topic = '/torcs/reset'
    reward_topic = '/torcs/reward'
    ros_update_rate = 1
    torcs_relaunch = 3

    def __init__(self,
                 vision=False,
                 throttle=True,
                 gear_change=False,
                 tanh_adjust=False,
                 msg_level=INFO):

        self._lock = _thread.allocate_lock()
        self._msg_level = msg_level
        self._runs = 0
        self._tanh_adjust = tanh_adjust
        self._current_step = 0
        self._current_score = 0
        self._terminated = True
        self._in_reset = False
        self._env = TorcsEnv(vision=vision, throttle=throttle,
                             gear_change=gear_change)

        self.log('Initialize node')
        # Initialize publishers
        self._state_publisher = rospy.Publisher(
            RosTorcsConnector.state_topic, Float32MultiArray, queue_size=1)

        self._terminate_publisher = rospy.Publisher(
            RosTorcsConnector.terminate_topic, Bool, queue_size=1)

        self._reward_publisher = rospy.Publisher(
            RosTorcsConnector.reward_topic, Float32, queue_size=1)

        # Initialize Subscribers
        self._action_subscriber = rospy.Subscriber(
            RosTorcsConnector.step_topic, Float32MultiArray, self.step)

        self._reset_subscriber = rospy.Subscriber(
            RosTorcsConnector.reset_topic, Bool, self.reset)

        rospy.init_node('RosTorcsEnvironment', anonymous=True)
        rate = rospy.Rate(RosTorcsConnector.ros_update_rate)
        self._listener = _thread.start_new_thread(rospy.spin, ())
        time.sleep(2)
        self.log('Ros node initialized')

    def is_terminated(self):
        return self._terminated

    def in_reset(self):
        return self._in_reset

    def make_relaunch(self):
        return self._runs % RosTorcsConnector.torcs_relaunch == 0

    def reset(self, msg=Bool(data=True)):
        self._lock.acquire()
        if msg.data is True:
            self.__in_reset = True
            self._score = 0
            self._current_step = 0
            obs = self._env.reset(relaunch=self.make_relaunch())
            state = self.filter_state(obs)
            self.log('New TORCS round starts...')
            self.log('Initial state: {}'.format(state), level=DEBUG)
            self._terminate_publisher.publish(Bool(False))
            self._state_publisher.publish(Float32MultiArray(data=state))
            self._runs += 1
            self._terminated = False
            self._in_reset = False
        self._lock.release()

    def filter_state(self, obs):
        return np.hstack((
            obs.angle,
            obs.track,
            obs.trackPos,
            obs.speedX,
            obs.speedY,
            obs.speedZ,
            obs.wheelSpinVel/100.0,
            obs.rpm
        )).astype('float32')

    def naNFilter(self, action):
        for i in range(len(action)):
            if np.isnan(action[i]):
                action[i] = 0
        return action

    def step(self, msg):
        if self._terminated or self._in_reset:
            self.log(
                'Discard action because no game running', level=INFO)
            return

        if self._lock.locked():
            self.log('Cannot keep up; Discard step')
            return

        self._lock.acquire()

        action = np.array(msg.data, dtype='float32')
        action = self.naNFilter(action)
        if self._tanh_adjust:
            action = self.adjust_action(action)

        obs, r, t, _ = self._env.step(action)
        s = self.filter_state(obs)
        r = np.float32(r)
        t = np.bool(t)
        self._current_step += 1

        self.log(
            'Step {},; Action: {} ; Rewaerd: {}, Resulting state: {}'
            .format(self._current_step, action, r, s, t), level=DEBUG)

        if t:
            self.log('Game terminated...')
            self._terminated = True

        state_msg = Float32MultiArray(data=s)
        self._state_publisher.publish(state_msg)
        self._terminate_publisher.publish(Bool(t))
        self._reward_publisher.publish(Float32(r))
        self._lock.release()

    def adjust_action(self, action):
        adj_action = np.zeros(shape=(3,), dtype='float32')
        adj_action[0] = action[0]
        adj_action[1] = 0.5 * action[1] + 0.5
        adj_action[2] = 0.5 * action[2] + 0.5
        return adj_action

    def log(self, msg, level=INFO):
        if self._msg_level <= level:
            print(msg)

    def shutdown(self):
        rospy.signal_shutdown('Shutdown')
