# (c) https://github.com/MontiCore/monticore  
import gym
import rospy
import time
import thread

from std_msgs.msg import Float32MultiArray, Bool, Int32, Float32
from gym import wrappers


class BaseEnvironment(object):
    def __init__(self):
        pass


class RosGymConnector(object):
    state_topic = '/gym/state'
    terminate_topic = '/gym/terminal'
    step_topic = '/gym/step'
    reset_topic = '/gym/reset'
    reward_topic = '/gym/reward'
    ros_update_rate = 10

    def __init__(self, env_str, verbose=True, render_per_step=1,
                 continuous=False, make_video=False):
        assert render_per_step >= 0, 'Render per steps needs to be positive'
        self.__env_str = env_str
        self.__verbose = verbose
        self.__render_per_step = render_per_step
        self.__env = gym.make(env_str)
        self.__make_video = make_video
        self.__last_game_score = 0
        if make_video:
            self.__env = wrappers.Monitor(
                self.__env, './videos/' + str(time.time()) + '-' +
                self.__env_str, force=True)

        self.__terminated = True
        self.__continuous = continuous

        self.__state_shape = self.__env.observation_space.shape
        self.__state_dim = len(self.__state_shape)
        self.__flat_state_dim = 1
        for i in range(self.__state_dim):
            self.__flat_state_dim =\
                self.__flat_state_dim * self.__state_shape[i]

        self.print_if_verbose('Initialize node')

        self.__state_publisher = rospy.Publisher(
            RosGymConnector.state_topic, Float32MultiArray, queue_size=1)
        self.__terminate_publisher = rospy.Publisher(
            RosGymConnector.terminate_topic, Bool, queue_size=1)
        self.__reward_publisher = rospy.Publisher(
            RosGymConnector.reward_topic, Float32, queue_size=1)

        if self.is_continuous:
            self.__action_subscriber = rospy.Subscriber(
                RosGymConnector.step_topic, Float32MultiArray, self.step)
        else:
            self.__action_subscriber = rospy.Subscriber(
                RosGymConnector.step_topic, Int32, self.step)

        self.__reset_subscriber = rospy.Subscriber(
            RosGymConnector.reset_topic, Bool, self.reset)

        self.__current_step = 0
        self.__score = 0

        rospy.init_node(
            'gymEnv{}'.format(env_str.replace('-', '_')), anonymous=True)
        rate = rospy.Rate(RosGymConnector.ros_update_rate)
        self.__listener = thread.start_new_thread(rospy.spin, ())
        time.sleep(2)
        self.print_if_verbose('Ros node initialized')

    @property
    def is_terminated(self):
        return self.__terminated

    @property
    def in_reset(self):
        return self.__in_reset

    @property
    def is_continuous(self):
        return self.__continuous

    @property
    def last_game_score(self):
        return self.__last_game_score

    def reset(self, msg=Bool(data=True)):
        if msg.data is True and self.is_terminated:
            self.__in_reset = True
            self.__score = 0
            self.__current_step = 0
            state = self.__env.reset()
            self.print_if_verbose('Game {} started'.format(self.__env_str))
            self.render()
            time.sleep(1)
            self.__terminate_publisher.publish(Bool(False))
            self.__state_publisher.publish(Float32MultiArray(
                data=self.flatten(state)))
            self.__reward_publisher.publish(Float32(0.0))
            self.__terminated = False
            self.__in_reset = False

    def step(self, msg):
        if not self.is_continuous:
            assert 0 <= msg.data < self.__env.action_space.n,\
                'Action {} is not in action space'.format(msg.data)

        if self.__terminated or self.__in_reset:
            self.print_if_verbose('Discard action because no game running')
            return

        action = msg.data

        s, r, t, _ = self.__env.step(action)
        self.render()
        self.__score += r
        self.__current_step += 1

        self.print_if_verbose('Action {} received; Current Score: {}'.format(
            action, self.__score))

        if t:
            print('Game terminated with score {}'.format(self.__score))
            self.__terminated = True
            self.__last_game_score = self.__score

        state_msg = Float32MultiArray(data=self.flatten(s))
        self.__state_publisher.publish(state_msg)
        self.__terminate_publisher.publish(Bool(t))
        self.__reward_publisher.publish(Float32(r))

    def render(self):
        if self.__render_per_step == 0 or self.__make_video:
            return
        if (self.__current_step % self.__render_per_step) == 0:
            self.__env.render()

    def flatten(self, state):
        if self.__state_dim > 1:
            return state.reshape((self.__flat_state_dim))
        else:
            return state

    def print_if_verbose(self, message):
        if self.__verbose:
            rospy.loginfo(message)

    def shutdown(self):
        self.__env.close()
        rospy.signal_shutdown('Shutdown')
