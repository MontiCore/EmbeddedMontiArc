from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2
import numpy as np

import logging
import os
import shutil
import sys

#TODO: Check whether class is needed
#class CNNCreator_VGG16:

module = None
_data_dir_ = "data/VGG16/"
_model_dir_ = "model/VGG16/"
_model_prefix_ = "VGG16"
_input_names_ = ['data']
_input_shapes_ = [(3,224,224)]
_output_names_ = ['predictions_label']

#TODO: Modify paths to make them dynamic
#For Windows
#INIT_NET = 'D:/Yeverino/git_projects/Caffe2_scripts/caffe2_ema_cnncreator/init_net'
#PREDICT_NET = 'D:/Yeverino/git_projects/Caffe2_scripts/caffe2_ema_cnncreator/predict_net'

#For Ubuntu
INIT_NET = '/home/carlos/Documents/git/Caffe2_scripts/caffe2_ema_cnncreator/init_net'
PREDICT_NET = '/home/carlos/Documents/git/Caffe2_scripts/caffe2_ema_cnncreator/predict_net'

#device_opts = core.DeviceOption(caffe2_pb2.CPU, 0) #for CPU processing
device_opts = core.DeviceOption(caffe2_pb2.CUDA, 0) #for GPU processing

#data and label are dummy at the moment
# randomly creates 30x30 patches of ones or zeros with label 1 and 0 respectively
def get_dummy_data(batchsize) :
	data = []
	label = []
	for i in range(batchsize) :
		r = np.random.randint(0, 2)
		if r==0 :
			d = np.zeros((1,30,30))
			l = 0
		else :
			d = np.ones((1,30,30))
			l = 1
		data.append(d)
		label.append(l)
	return np.array(data).astype('float32'), np.array(label).astype('int32')

def AddInput(model, batch_size):
	data, label = get_dummy_data(batch_size)
	print '\ndata:', data
	print '\nlabel:', label

	return data, label

def create_model(model, device_opts):
	with core.DeviceScope(device_opts):

		data, label = AddInput(model, batch_size=100)
		# data, output shape: {[3,224,224]}
		workspace.FeedBlob("data", data, device_option=device_opts)
		workspace.FeedBlob("label", label, device_option=device_opts)
  	
		conv1_ = brew.conv(model, 'data', 'conv1_', dim_in=1, dim_out=64, kernel=3, stride=1)
		# conv1_, output shape: {[64,224,224]}
		relu1_ = brew.relu(model, conv1_, conv1_)
  		conv2_ = brew.conv(model, relu1_, 'conv2_', dim_in=64, dim_out=64, kernel=3, stride=1)
		# conv2_, output shape: {[64,224,224]}
		relu2_ = brew.relu(model, conv2_, conv2_)
		pool2_ = brew.max_pool(model, relu2_, 'pool2_', kernel=2, stride=2)
		# pool2_, output shape: {[64,112,112]}
  		conv3_ = brew.conv(model, pool2_, 'conv3_', dim_in=64, dim_out=128, kernel=3, stride=1)
		# conv3_, output shape: {[128,112,112]}
		relu3_ = brew.relu(model, conv3_, conv3_)
  		conv4_ = brew.conv(model, relu3_, 'conv4_', dim_in=128, dim_out=128, kernel=3, stride=1)
		# conv4_, output shape: {[128,112,112]}
		relu4_ = brew.relu(model, conv4_, conv4_)
		pool4_ = brew.max_pool(model, relu4_, 'pool4_', kernel=2, stride=2)
		# pool4_, output shape: {[128,56,56]}
  		conv5_ = brew.conv(model, pool4_, 'conv5_', dim_in=128, dim_out=256, kernel=3, stride=1)
		# conv5_, output shape: {[256,56,56]}
		relu5_ = brew.relu(model, conv5_, conv5_)
  		conv6_ = brew.conv(model, relu5_, 'conv6_', dim_in=256, dim_out=256, kernel=3, stride=1)
		# conv6_, output shape: {[256,56,56]}
		relu6_ = brew.relu(model, conv6_, conv6_)
  		conv7_ = brew.conv(model, relu6_, 'conv7_', dim_in=256, dim_out=256, kernel=3, stride=1)
		# conv7_, output shape: {[256,56,56]}
		relu7_ = brew.relu(model, conv7_, conv7_)
		pool7_ = brew.max_pool(model, relu7_, 'pool7_', kernel=2, stride=2)
		# pool7_, output shape: {[256,28,28]}
  		conv8_ = brew.conv(model, pool7_, 'conv8_', dim_in=256, dim_out=512, kernel=3, stride=1)
		# conv8_, output shape: {[512,28,28]}
		relu8_ = brew.relu(model, conv8_, conv8_)
  		conv9_ = brew.conv(model, relu8_, 'conv9_', dim_in=512, dim_out=512, kernel=3, stride=1)
		# conv9_, output shape: {[512,28,28]}
		relu9_ = brew.relu(model, conv9_, conv9_)
  		conv10_ = brew.conv(model, relu9_, 'conv10_', dim_in=512, dim_out=512, kernel=3, stride=1)
		# conv10_, output shape: {[512,28,28]}
		relu10_ = brew.relu(model, conv10_, conv10_)
		pool10_ = brew.max_pool(model, relu10_, 'pool10_', kernel=2, stride=2)
		# pool10_, output shape: {[512,14,14]}
  		conv11_ = brew.conv(model, pool10_, 'conv11_', dim_in=512, dim_out=512, kernel=3, stride=1)
		# conv11_, output shape: {[512,14,14]}
		relu11_ = brew.relu(model, conv11_, conv11_)
  		conv12_ = brew.conv(model, relu11_, 'conv12_', dim_in=512, dim_out=512, kernel=3, stride=1)
		# conv12_, output shape: {[512,14,14]}
		relu12_ = brew.relu(model, conv12_, conv12_)
  		conv13_ = brew.conv(model, relu12_, 'conv13_', dim_in=512, dim_out=512, kernel=3, stride=1)
		# conv13_, output shape: {[512,14,14]}
		relu13_ = brew.relu(model, conv13_, conv13_)
		pool13_ = brew.max_pool(model, relu13_, 'pool13_', kernel=2, stride=2)
		# pool13_, output shape: {[512,7,7]}
		fc13_ = brew.fc(model, pool13_, 'fc13_', dim_in=512 * 7 * 7, dim_out=4096)
		# fc13_, output shape: {[4096,1,1]}
		relu14_ = brew.relu(model, fc13_, fc13_)
        dropout14_ = mx.symbol.Dropout(data=relu14_,
            p=0.5,
            name="dropout14_")
		fc14_ = brew.fc(model, dropout14_, 'fc14_', dim_in=4096, dim_out=4096)
		# fc14_, output shape: {[4096,1,1]}
		relu15_ = brew.relu(model, fc14_, fc14_)
        dropout15_ = mx.symbol.Dropout(data=relu15_,
            p=0.5,
            name="dropout15_")
		fc15_ = brew.fc(model, dropout15_, 'fc15_', dim_in=4096, dim_out=1000)
		# fc15_, output shape: {[1000,1,1]}
		predictions = brew.softmax(model, fc15_, 'predictions')

		model.net.AddExternalOutput(predictions)
		return predictions

# this adds the loss and optimizer
def add_training_operators(model, output, device_opts) :

	with core.DeviceScope(device_opts):
		xent = model.LabelCrossEntropy([output, "label"], 'xent')
		loss = model.AveragedLoss(xent, "loss")
		brew.accuracy(model, [output, "label"], "accuracy")

		model.AddGradientOperators([loss])
		opt = optimizer.build_sgd(model, base_learning_rate=0.01, policy="step", stepsize=1, gamma=0.999)  # , momentum=0.9

def train(INIT_NET, PREDICT_NET, epochs, batch_size, device_opts) :

	train_model= model_helper.ModelHelper(name="train_net")
	predictions = create_model(train_model, device_opts=device_opts)
	add_training_operators(train_model, predictions, device_opts=device_opts)
	with core.DeviceScope(device_opts):
		brew.add_weight_decay(train_model, 0.001)  # any effect???

	workspace.RunNetOnce(train_model.param_init_net)
	workspace.CreateNet(train_model.net)

	print '\ntraining for', epochs, 'epochs'

	for j in range(0, epochs):
		workspace.RunNet(train_model.net, 10)   # run for 10 times
		print str(j) + ': ' + 'loss ' + str(workspace.FetchBlob("loss")) + ' - ' + 'accuracy ' + str(workspace.FetchBlob("accuracy"))

	print 'training done'

	print '\nrunning test model'

	test_model= model_helper.ModelHelper(name="test_net", init_params=False)
	create_model(test_model, device_opts=device_opts)
	workspace.RunNetOnce(test_model.param_init_net)
	workspace.CreateNet(test_model.net, overwrite=True)

	data = np.zeros((1,1,30,30)).astype('float32')
	workspace.FeedBlob("data", data, device_option=device_opts)
	workspace.RunNet(test_model.net, 1)
	print "\nInput: zeros"
	print "Output:", workspace.FetchBlob("predictions") #TODO: Consider multiple output names
	print "Output class:", np.argmax(workspace.FetchBlob("predictions")) #TODO: Consider multiple output names

	data = np.ones((1,1,30,30)).astype('float32')
	workspace.FeedBlob("data", data, device_option=device_opts)
	workspace.RunNet(test_model.net, 1)
	print "\nInput: ones"
	print "Output:", workspace.FetchBlob("predictions") #TODO: Consider multiple output names
	print "Output class:", np.argmax(workspace.FetchBlob("predictions")) #TODO: Consider multiple output names

	print '\nsaving test model'

	save_net(INIT_NET, PREDICT_NET, test_model)

def save_net(init_net_path, predict_net_path, model):
	extra_params = []
	extra_blobs = []
	for blob in workspace.Blobs():
		name = str(blob)
		if name.endswith("_rm") or name.endswith("_riv"):
			extra_params.append(name)
			extra_blobs.append(workspace.FetchBlob(name))
	for name, blob in zip(extra_params, extra_blobs):
		model.params.append(name)

	init_net, predict_net = mobile_exporter.Export(
		workspace,
		model.net,
		model.params
	)

	print("Save the model to init_net.pb and predict_net.pb")
	with open(predict_net_path + '.pb', 'wb') as f:
		f.write(model.net._net.SerializeToString())
	with open(init_net_path + '.pb', 'wb') as f:
		f.write(init_net.SerializeToString())

	print("Save the mode to init_net.pbtxt and predict_net.pbtxt")
	with open(init_net_path + '.pbtxt', 'w') as f:
		f.write(str(init_net))

	with open(predict_net_path + '.pbtxt', 'w') as f:
		f.write(str(predict_net))

def load_net(init_net_path, predict_net_path, device_opts):
	init_def = caffe2_pb2.NetDef()
	with open(init_net_path + '.pb', 'rb') as f:
		init_def.ParseFromString(f.read())
		init_def.device_option.CopyFrom(device_opts)
		workspace.RunNetOnce(init_def.SerializeToString())

	net_def = caffe2_pb2.NetDef()
	with open(predict_net_path + '.pb', 'rb') as f:
		net_def.ParseFromString(f.read())
		net_def.device_option.CopyFrom(device_opts)
		workspace.CreateNet(net_def.SerializeToString(), overwrite=True)

train(INIT_NET, PREDICT_NET, epochs=20, batch_size=100, device_opts=device_opts)

print '\n********************************************'
print 'loading test model'
load_net(INIT_NET, PREDICT_NET, device_opts=device_opts)

data = np.ones((1,1,30,30)).astype('float32')
workspace.FeedBlob("data", data, device_option=device_opts)
workspace.RunNet('test_net', 1)

print "\nInput: ones"
print "Output:", workspace.FetchBlob("predictions") #TODO: Consider multiple output names
print "Output class:", np.argmax(workspace.FetchBlob("predictions")) #TODO: Consider multiple output names
