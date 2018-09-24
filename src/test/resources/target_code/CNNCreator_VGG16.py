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
#class CNNCreator_VGG16:

module = None
_data_dir_ = "data/VGG16/"
_model_dir_ = "model/VGG16/"
_model_prefix_ = "VGG16"
_input_names_ = ['data']
_input_shapes_ = [(3,224,224)]
_output_names_ = ['predictions_label']

EPOCHS     = 10000	# total training iterations
BATCH_SIZE = 256	# batch size for training
CONTEXT = 'gpu'
EVAL_METRIC = 'accuracy'
OPTIMIZER_TYPE = 'adam'
BASE_LEARNING_RATE = 0.001
WEIGHT_DECAY = 0.001
POLICY = 'fixed'
STEP_SIZE = 1
EPSILON = 1e-8
BETA1 = 0.9
BETA2 = 0.999
GAMMA = 0.999
MOMENTUM = 0.9


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

# Move into train function if test of deploy_net is removed
if CONTEXT == 'cpu':
    device_opts = core.DeviceOption(caffe2_pb2.CPU, 0)
    print("CPU mode selected")
elif CONTEXT == 'gpu':
    device_opts = core.DeviceOption(caffe2_pb2.CUDA, 0)
    print("GPU mode selected")

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
  	
		conv1_ = brew.conv(model, data, 'conv1_', dim_in=1, dim_out=64, kernel=3, stride=1)
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

		return predictions

# this adds the loss and optimizer
def add_training_operators(model, output, label, device_opts) :
	with core.DeviceScope(device_opts):
		xent = model.LabelCrossEntropy([output, label], 'xent')
		loss = model.AveragedLoss(xent, "loss")

		model.AddGradientOperators([loss])

        if OPTIMIZER_TYPE == 'adam':
            if POLICY == 'step':
                opt = optimizer.build_adam(model, base_learning_rate=BASE_LEARNING_RATE, policy=POLICY, stepsize=STEP_SIZE, beta1=BETA1, beta2=BETA2, epsilon=EPSILON)
            elif POLICY == 'fixed' or POLICY == 'inv':
                opt = optimizer.build_adam(model, base_learning_rate=BASE_LEARNING_RATE, policy=POLICY, beta1=BETA1, beta2=BETA2, epsilon=EPSILON)
            print("adam optimizer selected")
        elif OPTIMIZER_TYPE == 'sgd':
            if POLICY == 'step':
                opt = optimizer.build_sgd(model, base_learning_rate=BASE_LEARNING_RATE, policy=POLICY, stepsize=STEP_SIZE, gamma=GAMMA, momentum=MOMENTUM)
            elif POLICY == 'fixed' or POLICY == 'inv':
                opt = optimizer.build_sgd(model, base_learning_rate=BASE_LEARNING_RATE, policy=POLICY, gamma=GAMMA, momentum=MOMENTUM)
            print("sgd optimizer selected")
        elif OPTIMIZER_TYPE == 'rmsprop':
            if POLICY == 'step':
                opt = optimizer.build_rms_prop(model, base_learning_rate=BASE_LEARNING_RATE, policy=POLICY, stepsize=STEP_SIZE, decay=GAMMA, momentum=MOMENTUM, epsilon=EPSILON)
            elif POLICY == 'fixed' or POLICY == 'inv':
                opt = optimizer.build_rms_prop(model, base_learning_rate=BASE_LEARNING_RATE, policy=POLICY, decay=GAMMA, momentum=MOMENTUM, epsilon=EPSILON)
            print("rmsprop optimizer selected")
        elif OPTIMIZER_TYPE == 'adagrad':
            if POLICY == 'step':
                opt = optimizer.build_adagrad(model, base_learning_rate=BASE_LEARNING_RATE, policy=POLICY, stepsize=STEP_SIZE, decay=GAMMA, epsilon=EPSILON)
            elif POLICY == 'fixed' or POLICY == 'inv':
                opt = optimizer.build_adagrad(model, base_learning_rate=BASE_LEARNING_RATE, policy=POLICY, decay=GAMMA, epsilon=EPSILON)
            print("adagrad optimizer selected")

def add_accuracy(model, output, label, device_opts):
    with core.DeviceScope(device_opts):
        if EVAL_METRIC == 'accuracy':
            accuracy = brew.accuracy(model, [output, label], "accuracy")
        elif EVAL_METRIC == 'top_k_accuracy':
            accuracy = brew.accuracy(model, [output, label], "accuracy", top_k=3)
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
		brew.add_weight_decay(train_model, WEIGHT_DECAY)

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

	print("Saving deploy model")
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
print("Loading Deploy model")
load_net(INIT_NET, PREDICT_NET, device_opts=device_opts)

img = cv2.imread("3.jpg")                                   # Load test image
img = cv2.resize(img, (28,28))                              # Resize to 28x28
img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY )               # Covert to grayscale
img = img.reshape((1,1,28,28)).astype('float32')            # Reshape to (1,1,28,28)
workspace.FeedBlob("data", img, device_option=device_opts)  # FeedBlob
workspace.RunNet('deploy_net', num_iter=1)                  # Forward

print("\nInput: {}".format(img.shape))
pred = workspace.FetchBlob("predictions") #TODO: Consider multiple output names
print("Output: {}".format(pred))
print("Output class: {}".format(np.argmax(pred)))