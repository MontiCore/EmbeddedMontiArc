from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2
import numpy as np

import logging
import os
import shutil
#import h5py
import sys

#class CNNCreator_SimpleNetworkRelu:

module = None
_data_dir_ = "data/${tc.fullArchitectureName}/"
_model_dir_ = "model/${tc.fullArchitectureName}/"
_model_prefix_ = "${tc.architectureName}"
_input_names_ = [${tc.join(tc.architectureInputs, ",", "'", "'")}]
_input_shapes_ = [<#list tc.architecture.inputs as input>(${tc.join(input.definition.type.dimensions, ",")})</#list>]
_output_names_ = [${tc.join(tc.architectureOutputs, ",", "'", "_label'")}]

INIT_NET = 'D:/Yeverino/git_projects/Caffe2_scripts/caffe2_ema_cnncreator/init_net'
PREDICT_NET = 'D:/Yeverino/git_projects/Caffe2_scripts/caffe2_ema_cnncreator/predict_net'

#device_opts = core.DeviceOption(caffe2_pb2.CPU, 0)
device_opts = core.DeviceOption(caffe2_pb2.CUDA, 0)#' for GPU processing

# randomly creates 30x30 patches of ones or zeros with label 1 and 0 respectively
def get_data(batchsize) :
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

def create_model(model, device_opts):
	with core.DeviceScope(device_opts):

${tc.include(tc.architecture.body)}

# add loss and optimizer
def add_training_operators(softmax, model, device_opts) :

	with core.DeviceScope(device_opts):
		xent = model.LabelCrossEntropy([softmax, "label"], 'xent')
		loss = model.AveragedLoss(xent, "loss")
		brew.accuracy(model, [softmax, "label"], "accuracy")

		model.AddGradientOperators([loss])
		opt = optimizer.build_sgd(model, base_learning_rate=0.01, policy="step", stepsize=1, gamma=0.999)  # , momentum=0.9

def train(INIT_NET, PREDICT_NET, epochs, batch_size, device_opts) :

	data, label = get_data(batch_size)
	print '\ndata:', data
	print '\nlabel:', label

	workspace.FeedBlob("data", data, device_option=device_opts)
	workspace.FeedBlob("label", label, device_option=device_opts)

	train_model= model_helper.ModelHelper(name="train_net")
	softmax = create_model(train_model, device_opts=device_opts)
	add_training_operators(softmax, train_model, device_opts=device_opts)
	with core.DeviceScope(device_opts):
		brew.add_weight_decay(train_model, 0.001)  # any effect???

	workspace.RunNetOnce(train_model.param_init_net)
	workspace.CreateNet(train_model.net)

	print '\ntraining for', epochs, 'epochs'

	for j in range(0, epochs):
		data, label = get_data(batch_size)

		workspace.FeedBlob("data", data, device_option=device_opts)
		workspace.FeedBlob("label", label, device_option=device_opts)

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
	print "Output:", workspace.FetchBlob("out1") #TODO: pass output name
	print "Output class:", np.argmax(workspace.FetchBlob("out1")) #TODO: pass output name

	data = np.ones((1,1,30,30)).astype('float32')
	workspace.FeedBlob("data", data, device_option=device_opts)
	workspace.RunNet(test_model.net, 1)
	print "\nInput: ones"
	print "Output:", workspace.FetchBlob("out1") #TODO: pass output name
	print "Output class:", np.argmax(workspace.FetchBlob("out1")) #TODO: pass output name

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
print "Output:", workspace.FetchBlob("out1") #TODO: pass output name
print "Output class:", np.argmax(workspace.FetchBlob("out1")) #TODO: pass output name
