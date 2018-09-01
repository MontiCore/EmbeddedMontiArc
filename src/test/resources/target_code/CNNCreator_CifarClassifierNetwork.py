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
_data_dir_ = "data/CifarClassifierNetwork/"
_model_dir_ = "model/CifarClassifierNetwork/"
_model_prefix_ = "CifarClassifierNetwork"
_input_names_ = ['data']
_input_shapes_ = [(3,32,32)]
_output_names_ = ['softmax_label']

INIT_NET = 'D:/Yeverino/git_projects/Caffe2_scripts/caffe2_ema_cnncreator/init_net'
PREDICT_NET = 'D:/Yeverino/git_projects/Caffe2_scripts/caffe2_ema_cnncreator/predict_net'

#device_opts = core.DeviceOption(caffe2_pb2.CPU, 0)
device_opts = core.DeviceOption(caffe2_pb2.CUDA, 0)#' for GPU processing

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

#def create_model(model, data, device_opts): #data argument is dummy at the moment
def create_model(model, device_opts):
	with core.DeviceScope(device_opts):

		data, label = AddInput(model, batch_size=100)
		# data, output shape: {[3,32,32]}
		workspace.FeedBlob("data", data, device_option=device_opts)
		workspace.FeedBlob("label", label, device_option=device_opts)
  	
		conv2_1_ = brew.conv(model, 'data', 'conv2_1_', dim_in=1, dim_out=8, kernel=3, stride=1)
		# conv2_1_, output shape: {[8,32,32]}
        batchnorm2_1_ = mx.symbol.BatchNorm(data=conv2_1_,
            fix_gamma=True,
            name="batchnorm2_1_")
		relu2_1_ = brew.relu(model, batchnorm2_1_, batchnorm2_1_)
  		conv3_1_ = brew.conv(model, relu2_1_, 'conv3_1_', dim_in=8, dim_out=8, kernel=3, stride=1)
		# conv3_1_, output shape: {[8,32,32]}
        batchnorm3_1_ = mx.symbol.BatchNorm(data=conv3_1_,
            fix_gamma=True,
            name="batchnorm3_1_")
	
		conv2_2_ = brew.conv(model, 'data', 'conv2_2_', dim_in=1, dim_out=8, kernel=1, stride=1)
		# conv2_2_, output shape: {[8,32,32]}
        batchnorm2_2_ = mx.symbol.BatchNorm(data=conv2_2_,
            fix_gamma=True,
            name="batchnorm2_2_")
        add4_ = batchnorm3_1_ + batchnorm2_2_
		# add4_, output shape: {[8,32,32]}
		relu4_ = brew.relu(model, add4_, add4_)
  		conv5_1_ = brew.conv(model, relu4_, 'conv5_1_', dim_in=8, dim_out=16, kernel=3, stride=2)
		# conv5_1_, output shape: {[16,16,16]}
        batchnorm5_1_ = mx.symbol.BatchNorm(data=conv5_1_,
            fix_gamma=True,
            name="batchnorm5_1_")
		relu5_1_ = brew.relu(model, batchnorm5_1_, batchnorm5_1_)
  		conv6_1_ = brew.conv(model, relu5_1_, 'conv6_1_', dim_in=16, dim_out=16, kernel=3, stride=1)
		# conv6_1_, output shape: {[16,16,16]}
        batchnorm6_1_ = mx.symbol.BatchNorm(data=conv6_1_,
            fix_gamma=True,
            name="batchnorm6_1_")
		conv5_2_ = brew.conv(model, relu4_, 'conv5_2_', dim_in=8, dim_out=16, kernel=1, stride=2)
		# conv5_2_, output shape: {[16,16,16]}
        batchnorm5_2_ = mx.symbol.BatchNorm(data=conv5_2_,
            fix_gamma=True,
            name="batchnorm5_2_")
        add7_ = batchnorm6_1_ + batchnorm5_2_
		# add7_, output shape: {[16,16,16]}
		relu7_ = brew.relu(model, add7_, add7_)
  		conv8_1_ = brew.conv(model, relu7_, 'conv8_1_', dim_in=16, dim_out=16, kernel=3, stride=1)
		# conv8_1_, output shape: {[16,16,16]}
        batchnorm8_1_ = mx.symbol.BatchNorm(data=conv8_1_,
            fix_gamma=True,
            name="batchnorm8_1_")
		relu8_1_ = brew.relu(model, batchnorm8_1_, batchnorm8_1_)
  		conv9_1_ = brew.conv(model, relu8_1_, 'conv9_1_', dim_in=16, dim_out=16, kernel=3, stride=1)
		# conv9_1_, output shape: {[16,16,16]}
        batchnorm9_1_ = mx.symbol.BatchNorm(data=conv9_1_,
            fix_gamma=True,
            name="batchnorm9_1_")
        add10_ = batchnorm9_1_ + relu7_
		# add10_, output shape: {[16,16,16]}
		relu10_ = brew.relu(model, add10_, add10_)
  		conv11_1_ = brew.conv(model, relu10_, 'conv11_1_', dim_in=16, dim_out=16, kernel=3, stride=1)
		# conv11_1_, output shape: {[16,16,16]}
        batchnorm11_1_ = mx.symbol.BatchNorm(data=conv11_1_,
            fix_gamma=True,
            name="batchnorm11_1_")
		relu11_1_ = brew.relu(model, batchnorm11_1_, batchnorm11_1_)
  		conv12_1_ = brew.conv(model, relu11_1_, 'conv12_1_', dim_in=16, dim_out=16, kernel=3, stride=1)
		# conv12_1_, output shape: {[16,16,16]}
        batchnorm12_1_ = mx.symbol.BatchNorm(data=conv12_1_,
            fix_gamma=True,
            name="batchnorm12_1_")
        add13_ = batchnorm12_1_ + relu10_
		# add13_, output shape: {[16,16,16]}
		relu13_ = brew.relu(model, add13_, add13_)
  		conv14_1_ = brew.conv(model, relu13_, 'conv14_1_', dim_in=16, dim_out=32, kernel=3, stride=2)
		# conv14_1_, output shape: {[32,8,8]}
        batchnorm14_1_ = mx.symbol.BatchNorm(data=conv14_1_,
            fix_gamma=True,
            name="batchnorm14_1_")
		relu14_1_ = brew.relu(model, batchnorm14_1_, batchnorm14_1_)
  		conv15_1_ = brew.conv(model, relu14_1_, 'conv15_1_', dim_in=32, dim_out=32, kernel=3, stride=1)
		# conv15_1_, output shape: {[32,8,8]}
        batchnorm15_1_ = mx.symbol.BatchNorm(data=conv15_1_,
            fix_gamma=True,
            name="batchnorm15_1_")
		conv14_2_ = brew.conv(model, relu13_, 'conv14_2_', dim_in=16, dim_out=32, kernel=1, stride=2)
		# conv14_2_, output shape: {[32,8,8]}
        batchnorm14_2_ = mx.symbol.BatchNorm(data=conv14_2_,
            fix_gamma=True,
            name="batchnorm14_2_")
        add16_ = batchnorm15_1_ + batchnorm14_2_
		# add16_, output shape: {[32,8,8]}
		relu16_ = brew.relu(model, add16_, add16_)
  		conv17_1_ = brew.conv(model, relu16_, 'conv17_1_', dim_in=32, dim_out=32, kernel=3, stride=1)
		# conv17_1_, output shape: {[32,8,8]}
        batchnorm17_1_ = mx.symbol.BatchNorm(data=conv17_1_,
            fix_gamma=True,
            name="batchnorm17_1_")
		relu17_1_ = brew.relu(model, batchnorm17_1_, batchnorm17_1_)
  		conv18_1_ = brew.conv(model, relu17_1_, 'conv18_1_', dim_in=32, dim_out=32, kernel=3, stride=1)
		# conv18_1_, output shape: {[32,8,8]}
        batchnorm18_1_ = mx.symbol.BatchNorm(data=conv18_1_,
            fix_gamma=True,
            name="batchnorm18_1_")
        add19_ = batchnorm18_1_ + relu16_
		# add19_, output shape: {[32,8,8]}
		relu19_ = brew.relu(model, add19_, add19_)
  		conv20_1_ = brew.conv(model, relu19_, 'conv20_1_', dim_in=32, dim_out=32, kernel=3, stride=1)
		# conv20_1_, output shape: {[32,8,8]}
        batchnorm20_1_ = mx.symbol.BatchNorm(data=conv20_1_,
            fix_gamma=True,
            name="batchnorm20_1_")
		relu20_1_ = brew.relu(model, batchnorm20_1_, batchnorm20_1_)
  		conv21_1_ = brew.conv(model, relu20_1_, 'conv21_1_', dim_in=32, dim_out=32, kernel=3, stride=1)
		# conv21_1_, output shape: {[32,8,8]}
        batchnorm21_1_ = mx.symbol.BatchNorm(data=conv21_1_,
            fix_gamma=True,
            name="batchnorm21_1_")
        add22_ = batchnorm21_1_ + relu19_
		# add22_, output shape: {[32,8,8]}
		relu22_ = brew.relu(model, add22_, add22_)
  		conv23_1_ = brew.conv(model, relu22_, 'conv23_1_', dim_in=32, dim_out=64, kernel=3, stride=2)
		# conv23_1_, output shape: {[64,4,4]}
        batchnorm23_1_ = mx.symbol.BatchNorm(data=conv23_1_,
            fix_gamma=True,
            name="batchnorm23_1_")
		relu23_1_ = brew.relu(model, batchnorm23_1_, batchnorm23_1_)
  		conv24_1_ = brew.conv(model, relu23_1_, 'conv24_1_', dim_in=64, dim_out=64, kernel=3, stride=1)
		# conv24_1_, output shape: {[64,4,4]}
        batchnorm24_1_ = mx.symbol.BatchNorm(data=conv24_1_,
            fix_gamma=True,
            name="batchnorm24_1_")
		conv23_2_ = brew.conv(model, relu22_, 'conv23_2_', dim_in=32, dim_out=64, kernel=1, stride=2)
		# conv23_2_, output shape: {[64,4,4]}
        batchnorm23_2_ = mx.symbol.BatchNorm(data=conv23_2_,
            fix_gamma=True,
            name="batchnorm23_2_")
        add25_ = batchnorm24_1_ + batchnorm23_2_
		# add25_, output shape: {[64,4,4]}
		relu25_ = brew.relu(model, add25_, add25_)
  		conv26_1_ = brew.conv(model, relu25_, 'conv26_1_', dim_in=64, dim_out=64, kernel=3, stride=1)
		# conv26_1_, output shape: {[64,4,4]}
        batchnorm26_1_ = mx.symbol.BatchNorm(data=conv26_1_,
            fix_gamma=True,
            name="batchnorm26_1_")
		relu26_1_ = brew.relu(model, batchnorm26_1_, batchnorm26_1_)
  		conv27_1_ = brew.conv(model, relu26_1_, 'conv27_1_', dim_in=64, dim_out=64, kernel=3, stride=1)
		# conv27_1_, output shape: {[64,4,4]}
        batchnorm27_1_ = mx.symbol.BatchNorm(data=conv27_1_,
            fix_gamma=True,
            name="batchnorm27_1_")
        add28_ = batchnorm27_1_ + relu25_
		# add28_, output shape: {[64,4,4]}
		relu28_ = brew.relu(model, add28_, add28_)
  		conv29_1_ = brew.conv(model, relu28_, 'conv29_1_', dim_in=64, dim_out=64, kernel=3, stride=1)
		# conv29_1_, output shape: {[64,4,4]}
        batchnorm29_1_ = mx.symbol.BatchNorm(data=conv29_1_,
            fix_gamma=True,
            name="batchnorm29_1_")
		relu29_1_ = brew.relu(model, batchnorm29_1_, batchnorm29_1_)
  		conv30_1_ = brew.conv(model, relu29_1_, 'conv30_1_', dim_in=64, dim_out=64, kernel=3, stride=1)
		# conv30_1_, output shape: {[64,4,4]}
        batchnorm30_1_ = mx.symbol.BatchNorm(data=conv30_1_,
            fix_gamma=True,
            name="batchnorm30_1_")
        add31_ = batchnorm30_1_ + relu28_
		# add31_, output shape: {[64,4,4]}
		relu31_ = brew.relu(model, add31_, add31_)
        globalpooling31_ = mx.symbol.Pooling(data=relu31_,
            global_pool=True,
            kernel=(1,1),
            pool_type="avg",
            name="globalpooling31_")
		# globalpooling31_, output shape: {[64,1,1]}
		fc31_ = brew.fc(model, globalpooling31_, 'fc31_', dim_in=64, dim_out=128)
		# fc31_, output shape: {[128,1,1]}
        dropout31_ = mx.symbol.Dropout(data=fc31_,
            p=0.5,
            name="dropout31_")
		fc32_ = brew.fc(model, dropout31_, 'fc32_', dim_in=128, dim_out=10)
		# fc32_, output shape: {[10,1,1]}
		softmax = brew.softmax(model, fc32_, 'softmax')

		model.net.AddExternalOutput(softmax)
		return softmax

# add loss and optimizer
#def add_training_operators(model, output, label, device_opts) : #label argument is dummy at the moment
def add_training_operators(model, output, device_opts) :

	with core.DeviceScope(device_opts):
		xent = model.LabelCrossEntropy([output, "label"], 'xent')
		loss = model.AveragedLoss(xent, "loss")
		brew.accuracy(model, [output, "label"], "accuracy")

		model.AddGradientOperators([loss])
		opt = optimizer.build_sgd(model, base_learning_rate=0.01, policy="step", stepsize=1, gamma=0.999)  # , momentum=0.9

def train(INIT_NET, PREDICT_NET, epochs, batch_size, device_opts) :

	train_model= model_helper.ModelHelper(name="train_net")
	#data, label = AddInput(train_model, batch_size=100)
	#predictions = create_model(train_model, data, device_opts=device_opts)
	#add_training_operators(train_model, predictions, label, device_opts=device_opts)
	softmax = create_model(train_model, device_opts=device_opts)
	add_training_operators(train_model, softmax, device_opts=device_opts)
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
	#create_model(test_model, data, device_opts=device_opts)
	create_model(test_model, device_opts=device_opts)
	workspace.RunNetOnce(test_model.param_init_net)
	workspace.CreateNet(test_model.net, overwrite=True)

	data = np.zeros((1,1,30,30)).astype('float32')
	workspace.FeedBlob("data", data, device_option=device_opts)
	workspace.RunNet(test_model.net, 1)
	print "\nInput: zeros"
	print "Output:", workspace.FetchBlob("softmax") #TODO: Consider multiple output names
	print "Output class:", np.argmax(workspace.FetchBlob("softmax")) #TODO: Consider multiple output names

	data = np.ones((1,1,30,30)).astype('float32')
	workspace.FeedBlob("data", data, device_option=device_opts)
	workspace.RunNet(test_model.net, 1)
	print "\nInput: ones"
	print "Output:", workspace.FetchBlob("softmax") #TODO: Consider multiple output names
	print "Output class:", np.argmax(workspace.FetchBlob("softmax")) #TODO: Consider multiple output names

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
print "Output:", workspace.FetchBlob("softmax") #TODO: Consider multiple output names
print "Output class:", np.argmax(workspace.FetchBlob("softmax")) #TODO: Consider multiple output names
