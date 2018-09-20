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
#class ${tc.fileNameWithoutEnding}:

module = None
_data_dir_ = "data/${tc.fullArchitectureName}/"
_model_dir_ = "model/${tc.fullArchitectureName}/"
_model_prefix_ = "${tc.architectureName}"
_input_names_ = [${tc.join(tc.architectureInputs, ",", "'", "'")}]
_input_shapes_ = [<#list tc.architecture.inputs as input>(${tc.join(input.definition.type.dimensions, ",")})</#list>]
_output_names_ = [${tc.join(tc.architectureOutputs, ",", "'", "_label'")}]

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

${tc.include(tc.architecture.body)}

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
	${tc.join(tc.architectureOutputs, ",", "","")} = create_model(train_model, data, device_opts=device_opts)
	add_training_operators(train_model, ${tc.join(tc.architectureOutputs, ",", "","")}, label, device_opts=device_opts)
	add_accuracy(train_model, ${tc.join(tc.architectureOutputs, ",", "","")}, label, device_opts)
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
	${tc.join(tc.architectureOutputs, ",", "","")} = create_model(test_model, data, device_opts=device_opts)
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
workspace.FeedBlob("${tc.architectureInputs[0]}", img, device_option=device_opts)  # FeedBlob
workspace.RunNet('deploy_net', num_iter=1)                  # Forward

print("\nInput: {}".format(img.shape))
pred = workspace.FetchBlob("${tc.architectureOutputs[0]}") #TODO: Consider multiple output names
print("Output: {}".format(pred))
print("Output class: {}".format(np.argmax(pred)))