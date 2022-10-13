<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
<#setting number_format="computer">
<#assign config = configurations[0]>
import abc
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
<#if config.hasRewardFunction() >
import ${config.rewardFunctionComponentName}_executor

class RewardFunction(object):
    def __init__(self):
        self.__reward_wrapper = ${config.rewardFunctionComponentName}_executor.${config.rewardFunctionComponentName}_executor()
        self.__reward_wrapper.init()

    def reward(self, state, terminal):
        s = state.astype('${config.rewardFunctionStateParameter.dtype}')
        t = bool(terminal)
        inp = ${config.rewardFunctionComponentName}_executor.${config.rewardFunctionComponentName}_input()
        inp.${config.rewardFunctionStateParameter.name} = s
        inp.${config.rewardFunctionTerminalParameter.name} = t
        output = self.__reward_wrapper.execute(inp)
        return output.${config.rewardFunctionOutputName}


</#if>

class Environment:
    __metaclass__ = abc.ABCMeta

    def __init__(self):
<#if config.hasRewardFunction() >
        self._reward_function = RewardFunction()
<#else>
        pass
</#if>

    @abc.abstractmethod
    def reset(self):
        pass

    @abc.abstractmethod
    def step(self, action):
        pass

    @abc.abstractmethod
    def close(self):
        pass

<#if config.environment?? && config.environmentName == "gym">
import gym
class GymEnvironment(Environment):
    def __init__(self, env_name, **kwargs):
        super(GymEnvironment, self).__init__(**kwargs)
        self.__seed = 42
        self.__env = gym.make(env_name)
        self.__env.seed(self.__seed)

    @property
    def state_dim(self):
        return self.__env.observation_space.shape


    @property
    def number_of_actions(self):
        return self.__env.action_space.n

    @property
    def rewards_dtype(self):
        return 'float32'

    def reset(self):
        return self.__env.reset()

    def step(self, action):
<#if config.hasRewardFunction() >
        next_state, reward, terminal, info = self.__env.step(action)
        reward = self._reward_function.reward(next_state)
        return next_state, reward, terminal, info
<#else>
        return self.__env.step(action)
</#if>

    def close(self):
        self.__env.close()

    def action_space(self):
        self.__env.action_space

    def is_in_action_space(self, action):
        return self.__env.action_space.contains(action)

    def sample_action(self):
        return self.__env.action_space.sample()

    def render(self):
        self.__env.render()
<#else>
<#assign action_ros_datatype=config.discreteRlAlgorithm?string("Int32", "Float32MultiArray")>
import rospy
import sys
if sys.version.strip().split(".")[0] > '2':
    import _thread as thread
else:
    import thread
import numpy as np
import time
from std_msgs.msg import Int32MultiArray, Bool, Int32, MultiArrayDimension, Float32

class RosEnvironment(Environment):
    def __init__(self,
        ros_node_name='RosTrainingAgent',
        timeout_in_s=3000,
        state_topic='state',
        action_topic='action',
        reset_topic='reset',
        terminal_state_topic='terminal',
        reward_topic='reward'):
        super(RosEnvironment, self).__init__()
        self.__timeout_in_s = timeout_in_s
        self.__in_reset = False
        self.__waiting_for_state_update = False
        self.__waiting_for_terminal_update = False
        self.__last_received_state = 0
        self.__last_received_terminal = True
<#if config.hasRosRewardTopic()>
        self.__last_received_reward = 0.0
        self.__waiting_for_reward_update = False
</#if>

        rospy.loginfo("Initialize node {0}".format(ros_node_name))

        self.__step_publisher = rospy.Publisher(action_topic, ${action_ros_datatype}, queue_size=1)
        rospy.loginfo('Step Publisher initialized with topic {}'.format(action_topic))

        self.__reset_publisher = rospy.Publisher(reset_topic, Bool, queue_size=1)
        rospy.loginfo('Reset Publisher initialized with topic {}'.format(reset_topic))

        rospy.init_node(ros_node_name, anonymous=True)

        self.__state_subscriber = rospy.Subscriber(state_topic, Int32MultiArray, self.__state_callback)
        rospy.loginfo('State Subscriber registered with topic {}'.format(state_topic))

        self.__terminal_state_subscriber = rospy.Subscriber(terminal_state_topic, Bool, self.__terminal_state_callback)
        rospy.loginfo('Terminal State Subscriber registered with topic {}'.format(terminal_state_topic))
<#if config.hasRosRewardTopic()>

        self.__reward_subscriber = rospy.Subscriber(reward_topic, Float32, self.__reward_callback)
        rospy.loginfo('Reward Subscriber registered with topic {}'.format(reward_topic))
</#if>

        rate = rospy.Rate(10)

        thread.start_new_thread(rospy.spin, ())
        time.sleep(2)

    def reset(self):
        self.__in_reset = True
        reset_message = Bool()
        reset_message.data = True
        self.__waiting_for_state_update = True
        self.__waiting_for_terminal_update = False
        self.__waiting_for_reward_update = False
        self.__reset_publisher.publish(reset_message)
        self.__wait_for_new_state(self.__reset_publisher, reset_message)
        while self.__last_received_terminal:
            pass
        self.__in_reset = False
        return self.__last_received_state

    def step(self, action):
        action_rospy = ${action_ros_datatype}()
<#if config.continuousRlAlgorithm>
        assert len(action.shape) == 1
        action_rospy.layout.dim.append(MultiArrayDimension())
        action_rospy.layout.dim[0].label = 'action'
        action_rospy.layout.dim[0].size = ${config.actionDim[0]}
        action_rospy.layout.dim[0].stride = ${config.actionDim[0]}
</#if>
        action_rospy.data = action

        logger.debug('Send action: {}'.format(action))

        self.__waiting_for_state_update = True
        self.__waiting_for_terminal_update = True
<#if config.hasRosRewardTopic()>
        self.__waiting_for_reward_update = True
</#if>
        self.__step_publisher.publish(action_rospy)
        self.__wait_for_new_state(self.__step_publisher, action_rospy)
        next_state = self.__last_received_state
        terminal = self.__last_received_terminal
        reward = <#if config.hasRosRewardTopic()>self.__last_received_reward<#else>self.__calc_reward(next_state, terminal)</#if>

        logger.debug('Transition: ({}, {}, {}, {})'.format(action, reward, next_state, terminal))

        return next_state, reward, terminal, 0

    def __wait_for_new_state(self, publisher, msg):
        time_of_timeout = time.time() + self.__timeout_in_s
        timeout_counter = 0
        while(self.__waiting_for_state_update
              or self.__waiting_for_terminal_update<#if config.hasRosRewardTopic()> or self.__waiting_for_reward_update</#if>):
            is_timeout = (time.time() > time_of_timeout)
            if (is_timeout):
                if timeout_counter < 3:
                    rospy.logwarn("Timeout occured: Retry message")
                    publisher.publish(msg)
                    timeout_counter += 1
                    time_of_timeout = time.time() + self.__timeout_in_s
                else:
                    rospy.logerr("Timeout 3 times in a row: Terminate application")
                    exit()
            time.sleep(1/500)

    def close(self):
        rospy.signal_shutdown('Program ended!')

    def __state_callback(self, data):
        self.__last_received_state = np.array(data.data, dtype='int32').reshape((<#list config.stateDim as d>${d},</#list>))
        logger.debug('Received state: {}'.format(self.__last_received_state))
        self.__waiting_for_state_update = False

    def __terminal_state_callback(self, data):
        self.__last_received_terminal = np.bool(data.data)
        logger.debug('Received terminal flag: {}'.format(self.__last_received_terminal))
        self.__waiting_for_terminal_update = False

<#if config.hasRosRewardTopic()>
    def __reward_callback(self, data):
        self.__last_received_reward = np.float32(data.data)
        logger.debug('Received reward: {}'.format(self.__last_received_reward))
        self.__waiting_for_reward_update = False
<#else>
    def __calc_reward(self, state, terminal):
        # C++ Wrapper call
        return self._reward_function.reward(state, terminal)
</#if>
</#if>
