import rospy
from std_msgs.msg import String, UInt16MultiArray, Float32MultiArray, UInt8MultiArray, Bool, Float32
import numpy as np
import time
import mxnet as mx
from mxnet import nd, autograd, gluon
from mxnet.gluon.data.vision import transforms
import _thread
import sys



NAME = "cv.nn.AlexnetSmall"
OUTPUT = True
REWARD = False

class AffordanceComponent:
	def __init__(self):
		self.data = np.zeros((3, 64, 64)) #mx.nd.zeros((1, 3, 64 ,64))
		self.predictions = np.zeros(14)
		self.net = gluon.nn.SymbolBlock.imports(
			symbol_file="model/" + NAME + "/model_0_newest-symbol.json", \
			input_names=['data'], \
			param_file="model/" + NAME + "/model_0_newest-0000.params")
		self.speed = 0
		self.last_msg_received = time.time()
		self.new_data = False
		self.reward = 0

	def reset(self):
		self.data = np.zeros((3, 64, 64))
		self.predictions = np.zeros(14)
		self.speed = 0

	def callback(self, data):
		self.data = np.frombuffer(data.data, dtype=np.uint8).reshape((3, 64, 64))
		self.last_msg_received = time.time()
		self.new_data = True

	def setSpeed(self, data):
		sensors = data.data
		self.speed = sensors[-8]

	def setReward(self, data):
		self.reward = data.data

	def run_prediction(self):
		input = mx.nd.array(np.array([self.data]))
		preds = self.net.forward(input)[0].asnumpy()
		preds[-1] = self.speed
		self.predictions = preds


class ROSConnection:
	def __init__(self, component):
		self.component = component
		component.reset()
		self.vision_listener = rospy.Subscriber('/torcs/vision', UInt8MultiArray, component.callback, queue_size=1)
		self.sensor_listener = rospy.Subscriber('/torcs/state', Float32MultiArray, component.setSpeed, queue_size=1) # needed for SpeedX
		self.reset_listener = rospy.Subscriber('/torcs/reset', Bool, self.reset, queue_size=1)
		#self.terminate_listener = rospy.Subscriber('/torcs/terminal', Bool, self.reset, queue_size=1)
		if REWARD:
			self.reward_listener = rospy.Subscriber('/torcs/reward_env', Float32, component.setReward, queue_size=1)
			self.reward_publisher = rospy.Publisher('/torcs/reward', Float32, queue_size=1)

		self.predictions_publisher = rospy.Publisher('/torcs/affordance', Float32MultiArray, queue_size=1)
		self.lock = _thread.allocate_lock()
		rospy.init_node('AffordanceComponent', anonymous=True)
		self.rate = rospy.Rate(5)
		self.listener_thread = _thread.start_new_thread(rospy.spin, ())
		self.prediction_thread = _thread.start_new_thread(self.send_predictions, ())
		print("ROS node initialized")                
		self.running = True
		self.reset_cnt = 0

	def reset(self, data): 
		self.component.reset()
		return

	def send_predictions(self):
		while True:
			self.predictions_publisher.publish(Float32MultiArray(data=self.component.predictions))
			if REWARD:
				self.reward_publisher.publish(Float32(data=self.component.reward))
			self.rate.sleep()

	def run(self):
		last_delay = time.time()
		linebreak = True
		delay = 0
		while not rospy.is_shutdown():
			if self.component.new_data:
				self.component.run_prediction()
				delay = time.time() - self.component.last_msg_received
				self.component.new_data = False
			if 1 >= delay >= 0.2 and OUTPUT:
				if time.time() - last_delay <= 0.2:
					sys.stdout.write("\rCant keep up. Output delay: %.3f" % (delay))
					sys.stdout.flush()
					linebreak = True
				else:
					if linebreak:
						print()
					print("Cant keep up. Output delay: %.3f" % (delay))
					linebreak = False
				last_delay = time.time()


if __name__ == '__main__':
	component = AffordanceComponent()
	connection = ROSConnection(component)
	connection.run()
