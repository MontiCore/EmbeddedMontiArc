from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2
import numpy as np

import logging
import os
import shutil
import sys
import cv2

#TODO: Check whether class is needed
#class CNNCreator_Alexnet:

module = None
_data_dir_ = "data/Alexnet/"
_model_dir_ = "model/Alexnet/"
_model_prefix_ = "Alexnet"
_input_names_ = ['data']
_input_shapes_ = [(3,224,224)]
_output_names_ = ['predictions_label']

EPOCHS     = 10000	# total training iterations
BATCH_SIZE = 256	# batch size for training

CURRENT_FOLDER      = os.path.join('./')
DATA_FOLDER         = os.path.join(CURRENT_FOLDER, 'data')
ROOT_FOLDER         = os.path.join(CURRENT_FOLDER, 'model')

#TODO: Modify paths to make them dynamic
#For Windows
#INIT_NET = 'D:/Yeverino/git_projects/Caffe2_scripts/caffe2_ema_cnncreator/init_net'
#PREDICT_NET = 'D:/Yeverino/git_projects/Caffe2_scripts/caffe2_ema_cnncreator/predict_net'

#For Ubuntu
INIT_NET = './model/init_net'
PREDICT_NET = './model/predict_net'

#device_opts = core.DeviceOption(caffe2_pb2.CPU, 0) #for CPU processing
device_opts = core.DeviceOption(caffe2_pb2.CUDA, 0) #for GPU processing

def add_input(model, batch_size, db, db_type, device_opts):
    with core.DeviceScope(device_opts):
        # load the data
        data_uint8, label = brew.db_input(
            model,
            blobs_out=["data_uint8", "label"],
            batch_size=batch_size,
            db=db,
            db_type=db_type,
        )
        # cast the data to float
        data = model.Cast(data_uint8, "data", to=core.DataType.FLOAT)

        # scale data from [0,255] down to [0,1]
        data = model.Scale(data, data, scale=float(1./256))

        # don't need the gradient for the backward pass
        data = model.StopGradient(data, data)
        return data, label

def create_model(model, data, device_opts):
	with core.DeviceScope(device_opts):

		data = data
		# data, output shape: {[3,224,224]}
  	
		conv1_ = brew.conv(model, 'data', 'conv1_', dim_in=1, dim_out=96, kernel=11, stride=4)
		# conv1_, output shape: {[96,55,55]}
        lrn1_ = mx.symbol.LRN(data=conv1_,
            alpha=0.0001,
            beta=0.75,
            knorm=2,
            nsize=5,
            name="lrn1_")
		pool1_ = brew.max_pool(model, lrn1_, 'pool1_', kernel=3, stride=2)
		# pool1_, output shape: {[96,27,27]}
		relu1_ = brew.relu(model, pool1_, pool1_)
        split1_ = mx.symbol.split(data=relu1_,
            num_outputs=2,
            axis=1,
            name="split1_")
		# split1_, output shape: {[48,27,27][48,27,27]}
        get2_1_ = split1_[0]
  		conv2_1_ = brew.conv(model, get2_1_, 'conv2_1_', dim_in=48, dim_out=128, kernel=5, stride=1)
		# conv2_1_, output shape: {[128,27,27]}
        lrn2_1_ = mx.symbol.LRN(data=conv2_1_,
            alpha=0.0001,
            beta=0.75,
            knorm=2,
            nsize=5,
            name="lrn2_1_")
		pool2_1_ = brew.max_pool(model, lrn2_1_, 'pool2_1_', kernel=3, stride=2)
		# pool2_1_, output shape: {[128,13,13]}
		relu2_1_ = brew.relu(model, pool2_1_, pool2_1_)
        get2_2_ = split1_[1]
  		conv2_2_ = brew.conv(model, get2_2_, 'conv2_2_', dim_in=48, dim_out=128, kernel=5, stride=1)
		# conv2_2_, output shape: {[128,27,27]}
        lrn2_2_ = mx.symbol.LRN(data=conv2_2_,
            alpha=0.0001,
            beta=0.75,
            knorm=2,
            nsize=5,
            name="lrn2_2_")
		pool2_2_ = brew.max_pool(model, lrn2_2_, 'pool2_2_', kernel=3, stride=2)
		# pool2_2_, output shape: {[128,13,13]}
		relu2_2_ = brew.relu(model, pool2_2_, pool2_2_)
        concatenate3_ = mx.symbol.concat(relu2_1_, relu2_2_,
            dim=1,
            name="concatenate3_")
		# concatenate3_, output shape: {[256,13,13]}
  		conv3_ = brew.conv(model, concatenate3_, 'conv3_', dim_in=256, dim_out=384, kernel=3, stride=1)
		# conv3_, output shape: {[384,13,13]}
		relu3_ = brew.relu(model, conv3_, conv3_)
        split3_ = mx.symbol.split(data=relu3_,
            num_outputs=2,
            axis=1,
            name="split3_")
		# split3_, output shape: {[192,13,13][192,13,13]}
        get4_1_ = split3_[0]
  		conv4_1_ = brew.conv(model, get4_1_, 'conv4_1_', dim_in=192, dim_out=192, kernel=3, stride=1)
		# conv4_1_, output shape: {[192,13,13]}
		relu4_1_ = brew.relu(model, conv4_1_, conv4_1_)
  		conv5_1_ = brew.conv(model, relu4_1_, 'conv5_1_', dim_in=192, dim_out=128, kernel=3, stride=1)
		# conv5_1_, output shape: {[128,13,13]}
		pool5_1_ = brew.max_pool(model, conv5_1_, 'pool5_1_', kernel=3, stride=2)
		# pool5_1_, output shape: {[128,6,6]}
		relu5_1_ = brew.relu(model, pool5_1_, pool5_1_)
        get4_2_ = split3_[1]
  		conv4_2_ = brew.conv(model, get4_2_, 'conv4_2_', dim_in=192, dim_out=192, kernel=3, stride=1)
		# conv4_2_, output shape: {[192,13,13]}
		relu4_2_ = brew.relu(model, conv4_2_, conv4_2_)
  		conv5_2_ = brew.conv(model, relu4_2_, 'conv5_2_', dim_in=192, dim_out=128, kernel=3, stride=1)
		# conv5_2_, output shape: {[128,13,13]}
		pool5_2_ = brew.max_pool(model, conv5_2_, 'pool5_2_', kernel=3, stride=2)
		# pool5_2_, output shape: {[128,6,6]}
		relu5_2_ = brew.relu(model, pool5_2_, pool5_2_)
        concatenate6_ = mx.symbol.concat(relu5_1_, relu5_2_,
            dim=1,
            name="concatenate6_")
		# concatenate6_, output shape: {[256,6,6]}
		fc6_ = brew.fc(model, concatenate6_, 'fc6_', dim_in=256 * 6 * 6, dim_out=4096)
		# fc6_, output shape: {[4096,1,1]}
		relu6_ = brew.relu(model, fc6_, fc6_)
        dropout6_ = mx.symbol.Dropout(data=relu6_,
            p=0.5,
            name="dropout6_")
		fc7_ = brew.fc(model, dropout6_, 'fc7_', dim_in=4096, dim_out=4096)
		# fc7_, output shape: {[4096,1,1]}
		relu7_ = brew.relu(model, fc7_, fc7_)
        dropout7_ = mx.symbol.Dropout(data=relu7_,
            p=0.5,
            name="dropout7_")
		fc8_ = brew.fc(model, dropout7_, 'fc8_', dim_in=4096, dim_out=10)
		# fc8_, output shape: {[10,1,1]}
		predictions = brew.softmax(model, fc8_, 'predictions')

		return predictions

# this adds the loss and optimizer
def add_training_operators(model, output, label, device_opts) :
	with core.DeviceScope(device_opts):
		xent = model.LabelCrossEntropy([output, label], 'xent')
		loss = model.AveragedLoss(xent, "loss")

		model.AddGradientOperators([loss])
		opt = optimizer.build_sgd(model, base_learning_rate=0.01, policy="step", stepsize=1, gamma=0.999)  # , momentum=0.9

def add_accuracy(model, output, label, device_opts):
    with core.DeviceScope(device_opts):
        accuracy = brew.accuracy(model, [output, label], "accuracy")
        return accuracy

def train(INIT_NET, PREDICT_NET, epochs, batch_size, device_opts) :

	workspace.ResetWorkspace(ROOT_FOLDER)

	arg_scope = {"order": "NCHW"}
	# == Training model ==
	train_model= model_helper.ModelHelper(name="train_net", arg_scope=arg_scope)
	data, label = add_input(train_model, batch_size=batch_size, db=os.path.join(DATA_FOLDER, 'mnist-train-nchw-lmdb'), db_type='lmdb', device_opts=device_opts)
	predictions = create_model(train_model, data, device_opts=device_opts)
	add_training_operators(train_model, predictions, label, device_opts=device_opts)
	add_accuracy(train_model, predictions, label, device_opts)
	with core.DeviceScope(device_opts):
		brew.add_weight_decay(train_model, 0.001)  # any effect???

	# Initialize and create the training network
	workspace.RunNetOnce(train_model.param_init_net)
	workspace.CreateNet(train_model.net, overwrite=True)

	# Main Training Loop
	print("== Starting Training for " + str(epochs) + " epochs ==")
	for j in range(0, epochs):
		workspace.RunNet(train_model.net)
		if j % 50 == 0:
			print 'Iter: ' + str(j) + ': ' + 'Loss ' + str(workspace.FetchBlob("loss")) + ' - ' + 'Accuracy ' + str(workspace.FetchBlob('accuracy'))
	print("Training done")

	print("== Running Test model ==")
	# == Testing model. ==
	test_model= model_helper.ModelHelper(name="test_net", arg_scope=arg_scope, init_params=False)
	data, label = add_input(test_model, batch_size=100, db=os.path.join(DATA_FOLDER, 'mnist-test-nchw-lmdb'), db_type='lmdb', device_opts=device_opts)
	predictions = create_model(test_model, data, device_opts=device_opts)
	add_accuracy(test_model, predictions, label, device_opts)
	workspace.RunNetOnce(test_model.param_init_net)
	workspace.CreateNet(test_model.net, overwrite=True)

	# Main Testing Loop
	# batch size:        100
	# iteration:         100
	# total test images: 10000
	test_accuracy = np.zeros(100)
	for i in range(100):
		# Run a forward pass of the net on the current batch
		workspace.RunNet(test_model.net)
		# Collect the batch accuracy from the workspace
		test_accuracy[i] = workspace.FetchBlob('accuracy')

	print('Test_accuracy: {:.4f}'.format(test_accuracy.mean()))

	# == Deployment model. ==
	# We simply need the main AddModel part.
	deploy_model = model_helper.ModelHelper(name="deploy_net", arg_scope=arg_scope, init_params=False)
	create_model(deploy_model, "data", device_opts)

	print("Saving test model")
	save_net(INIT_NET, PREDICT_NET, deploy_model)

def save_net(init_net_path, predict_net_path, model):

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

	print("Save the model to init_net.pbtxt and predict_net.pbtxt")
	with open(init_net_path + '.pbtxt', 'w') as f:
		f.write(str(init_net))
	with open(predict_net_path + '.pbtxt', 'w') as f:
		f.write(str(predict_net))
	print("== Saved init_net and predict_net ==")

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
	print("== Loaded init_net and predict_net ==")

train(INIT_NET, PREDICT_NET, epochs=EPOCHS, batch_size=BATCH_SIZE, device_opts=device_opts)

print '\n********************************************'
print("Loading Test model")
load_net(INIT_NET, PREDICT_NET, device_opts=device_opts)

img = cv2.imread("3.jpg")                                   # Load test image
img = cv2.resize(img, (28,28))                              # Resize to 28x28
img = cv2.cvtColor( img, cv2.COLOR_RGB2GRAY )               # Covert to grayscale
img = img.reshape((1,1,28,28)).astype('float32')            # Reshape to (1,1,28,28)
workspace.FeedBlob("data", img, device_option=device_opts)  # FeedBlob
workspace.RunNet('deploy_net', num_iter=1)                  # Forward

print("\nInput: {}".format(img.shape))
pred = workspace.FetchBlob("predictions") #TODO: Consider multiple output names
print("Output: {}".format(pred))
print("Output class: {}".format(np.argmax(pred)))