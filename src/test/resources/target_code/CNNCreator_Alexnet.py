# (c) https://github.com/MontiCore/monticore  
from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2
import numpy as np
import math
import logging
import os
import sys
import lmdb

class CNNCreator_Alexnet:

    module = None
    _current_dir_ = os.path.join('./')
    _data_dir_    = os.path.join(_current_dir_, 'data/Alexnet')
    _model_dir_   = os.path.join(_current_dir_, 'model', 'Alexnet')

    _init_net_    = os.path.join(_model_dir_, 'init_net.pb')
    _predict_net_ = os.path.join(_model_dir_, 'predict_net.pb')

    def get_total_num_iter(self, num_epoch, batch_size, dataset_size):
        #Force floating point calculation
        batch_size_float = float(batch_size)
        dataset_size_float = float(dataset_size)

        iterations_float = math.ceil(num_epoch*(dataset_size_float/batch_size_float))
        iterations_int = int(iterations_float)

        return iterations_int


    def add_input(self, model, batch_size, db, db_type, device_opts):
        with core.DeviceScope(device_opts):
            if not os.path.isdir(db):
                logging.error("Data loading failure. Directory '" + os.path.abspath(db) + "' does not exist.")
                sys.exit(1)
            elif not (os.path.isfile(os.path.join(db, 'data.mdb')) and os.path.isfile(os.path.join(db, 'lock.mdb'))):
                logging.error("Data loading failure. Directory '" + os.path.abspath(db) + "' does not contain lmdb files.")
                sys.exit(1)

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

            dataset_size = int (lmdb.open(db).stat()['entries'])

            return data, label, dataset_size

    def create_model(self, model, data, device_opts, is_test):
    	with core.DeviceScope(device_opts):

    		data = data
    		# data, output shape: {[3,224,224]}
    		conv1_ = brew.conv(model, data, 'conv1_', dim_in=3, dim_out=96, kernel=11, stride=4)
    		# conv1_, output shape: {[96,55,55]}
    		lrn1_ = brew.lrn(model, conv1_, 'lrn1_', size=5, alpha=0.0001, beta=0.75, bias=2.0)
    		pool1_ = brew.max_pool(model, lrn1_, 'pool1_', kernel=3, stride=2)
    		# pool1_, output shape: {[96,27,27]}
    		relu1_ = brew.relu(model, pool1_, pool1_)
    		conv2_1_ = brew.conv(model, get2_1_, 'conv2_1_', dim_in=48, dim_out=128, kernel=5, stride=1, pad=1)
    		# conv2_1_, output shape: {[128,27,27]}
    		lrn2_1_ = brew.lrn(model, conv2_1_, 'lrn2_1_', size=5, alpha=0.0001, beta=0.75, bias=2.0)
    		pool2_1_ = brew.max_pool(model, lrn2_1_, 'pool2_1_', kernel=3, stride=2)
    		# pool2_1_, output shape: {[128,13,13]}
    		relu2_1_ = brew.relu(model, pool2_1_, pool2_1_)
    		conv2_2_ = brew.conv(model, get2_2_, 'conv2_2_', dim_in=48, dim_out=128, kernel=5, stride=1, pad=1)
    		# conv2_2_, output shape: {[128,27,27]}
    		lrn2_2_ = brew.lrn(model, conv2_2_, 'lrn2_2_', size=5, alpha=0.0001, beta=0.75, bias=2.0)
    		pool2_2_ = brew.max_pool(model, lrn2_2_, 'pool2_2_', kernel=3, stride=2)
    		# pool2_2_, output shape: {[128,13,13]}
    		relu2_2_ = brew.relu(model, pool2_2_, pool2_2_)
    		conv3_ = brew.conv(model, concatenate3_, 'conv3_', dim_in=256, dim_out=384, kernel=3, stride=1, pad=1)
    		# conv3_, output shape: {[384,13,13]}
    		relu3_ = brew.relu(model, conv3_, conv3_)
    		conv4_1_ = brew.conv(model, get4_1_, 'conv4_1_', dim_in=192, dim_out=192, kernel=3, stride=1, pad=1)
    		# conv4_1_, output shape: {[192,13,13]}
    		relu4_1_ = brew.relu(model, conv4_1_, conv4_1_)
    		conv5_1_ = brew.conv(model, relu4_1_, 'conv5_1_', dim_in=192, dim_out=128, kernel=3, stride=1, pad=1)
    		# conv5_1_, output shape: {[128,13,13]}
    		pool5_1_ = brew.max_pool(model, conv5_1_, 'pool5_1_', kernel=3, stride=2)
    		# pool5_1_, output shape: {[128,6,6]}
    		relu5_1_ = brew.relu(model, pool5_1_, pool5_1_)
    		conv4_2_ = brew.conv(model, get4_2_, 'conv4_2_', dim_in=192, dim_out=192, kernel=3, stride=1, pad=1)
    		# conv4_2_, output shape: {[192,13,13]}
    		relu4_2_ = brew.relu(model, conv4_2_, conv4_2_)
    		conv5_2_ = brew.conv(model, relu4_2_, 'conv5_2_', dim_in=192, dim_out=128, kernel=3, stride=1, pad=1)
    		# conv5_2_, output shape: {[128,13,13]}
    		pool5_2_ = brew.max_pool(model, conv5_2_, 'pool5_2_', kernel=3, stride=2)
    		# pool5_2_, output shape: {[128,6,6]}
    		relu5_2_ = brew.relu(model, pool5_2_, pool5_2_)
    		fc6_ = brew.fc(model, concatenate6_, 'fc6_', dim_in=256 * 6 * 6, dim_out=4096)
    		# fc6_, output shape: {[4096,1,1]}
    		relu6_ = brew.relu(model, fc6_, fc6_)
    		dropout6_ = brew.dropout(model, relu6_, 'dropout6_', ratio=0.5, is_test=False)
    		fc7_ = brew.fc(model, dropout6_, 'fc7_', dim_in=4096, dim_out=4096)
    		# fc7_, output shape: {[4096,1,1]}
    		relu7_ = brew.relu(model, fc7_, fc7_)
    		dropout7_ = brew.dropout(model, relu7_, 'dropout7_', ratio=0.5, is_test=False)
    		fc8_ = brew.fc(model, dropout7_, 'fc8_', dim_in=4096, dim_out=10)
    		# fc8_, output shape: {[10,1,1]}
    		predictions = brew.softmax(model, fc8_, 'predictions')

    		return predictions

    # this adds the loss and optimizer
    def add_training_operators(self, model, output, label, device_opts, opt_type, base_learning_rate, policy, stepsize, epsilon, beta1, beta2, gamma, momentum) :
    	with core.DeviceScope(device_opts):
    		xent = model.LabelCrossEntropy([output, label], 'xent')
    		loss = model.AveragedLoss(xent, "loss")

    		model.AddGradientOperators([loss])

    		if opt_type == 'adam':
    		    if policy == 'step':
    		        opt = optimizer.build_adam(model, base_learning_rate=base_learning_rate, policy=policy, stepsize=stepsize, beta1=beta1, beta2=beta2, epsilon=epsilon)
    		    elif policy == 'fixed' or policy == 'inv':
    		        opt = optimizer.build_adam(model, base_learning_rate=base_learning_rate, policy=policy, beta1=beta1, beta2=beta2, epsilon=epsilon)
    		    print("adam optimizer selected")
    		elif opt_type == 'sgd':
    		    if policy == 'step':
    		        opt = optimizer.build_sgd(model, base_learning_rate=base_learning_rate, policy=policy, stepsize=stepsize, gamma=gamma, momentum=momentum)
    		    elif policy == 'fixed' or policy == 'inv':
    		        opt = optimizer.build_sgd(model, base_learning_rate=base_learning_rate, policy=policy, gamma=gamma, momentum=momentum)
    		    print("sgd optimizer selected")
    		elif opt_type == 'rmsprop':
    		    if policy == 'step':
    		        opt = optimizer.build_rms_prop(model, base_learning_rate=base_learning_rate, policy=policy, stepsize=stepsize, decay=gamma, momentum=momentum, epsilon=epsilon)
    		    elif policy == 'fixed' or policy == 'inv':
    		        opt = optimizer.build_rms_prop(model, base_learning_rate=base_learning_rate, policy=policy, decay=gamma, momentum=momentum, epsilon=epsilon)
    		    print("rmsprop optimizer selected")
    		elif opt_type == 'adagrad':
    		    if policy == 'step':
    		        opt = optimizer.build_adagrad(model, base_learning_rate=base_learning_rate, policy=policy, stepsize=stepsize, decay=gamma, epsilon=epsilon)
    		    elif policy == 'fixed' or policy == 'inv':
    		        opt = optimizer.build_adagrad(model, base_learning_rate=base_learning_rate, policy=policy, decay=gamma, epsilon=epsilon)
    		    print("adagrad optimizer selected")

    def add_accuracy(self, model, output, label, device_opts, eval_metric):
        with core.DeviceScope(device_opts):
            if eval_metric == 'accuracy':
                accuracy = brew.accuracy(model, [output, label], "accuracy")
            elif eval_metric == 'top_k_accuracy':
                accuracy = brew.accuracy(model, [output, label], "accuracy", top_k=3)
            return accuracy

    def train(self, num_epoch=1000, batch_size=64, context='gpu', eval_metric='accuracy', opt_type='adam', base_learning_rate=0.001, weight_decay=0.001, policy='fixed', stepsize=1, epsilon=1E-8, beta1=0.9, beta2=0.999, gamma=0.999, momentum=0.9) :
        if context == 'cpu':
            device_opts = core.DeviceOption(caffe2_pb2.CPU, 0)
            print("CPU mode selected")
        elif context == 'gpu':
            device_opts = core.DeviceOption(caffe2_pb2.CUDA, 0)
            print("GPU mode selected")

    	workspace.ResetWorkspace(self._model_dir_)

    	arg_scope = {"order": "NCHW"}
    	# == Training model ==
    	train_model= model_helper.ModelHelper(name="train_net", arg_scope=arg_scope)
    	data, label, train_dataset_size = self.add_input(train_model, batch_size=batch_size, db=os.path.join(self._data_dir_, 'train_lmdb'), db_type='lmdb', device_opts=device_opts)
    	predictions = self.create_model(train_model, data, device_opts=device_opts, is_test=False)
    	self.add_training_operators(train_model, predictions, label, device_opts, opt_type, base_learning_rate, policy, stepsize, epsilon, beta1, beta2, gamma, momentum)
    	self.add_accuracy(train_model, predictions, label, device_opts, eval_metric)
    	with core.DeviceScope(device_opts):
    		brew.add_weight_decay(train_model, weight_decay)

    	# Initialize and create the training network
    	workspace.RunNetOnce(train_model.param_init_net)
    	workspace.CreateNet(train_model.net, overwrite=True)

    	# Main Training Loop
    	iterations = self.get_total_num_iter(num_epoch, batch_size, train_dataset_size)
        print("** Starting Training for " + str(num_epoch) + " epochs = " + str(iterations) + " iterations **")
    	for i in range(iterations):
    		workspace.RunNet(train_model.net)
    		if i % 50 == 0:
    			print 'Iter ' + str(i) + ': ' + 'Loss ' + str(workspace.FetchBlob("loss")) + ' - ' + 'Accuracy ' + str(workspace.FetchBlob('accuracy'))
    	print("Training done")

    	print("== Running Test model ==")
    	# == Testing model. ==
    	test_model= model_helper.ModelHelper(name="test_net", arg_scope=arg_scope, init_params=False)
    	data, label, test_dataset_size = self.add_input(test_model, batch_size=batch_size, db=os.path.join(self._data_dir_, 'test_lmdb'), db_type='lmdb', device_opts=device_opts)
    	predictions = self.create_model(test_model, data, device_opts=device_opts, is_test=True)
    	self.add_accuracy(test_model, predictions, label, device_opts, eval_metric)
    	workspace.RunNetOnce(test_model.param_init_net)
    	workspace.CreateNet(test_model.net, overwrite=True)

    	# Main Testing Loop
    	test_accuracy = np.zeros(test_dataset_size/batch_size)
    	for i in range(test_dataset_size/batch_size):
    		# Run a forward pass of the net on the current batch
    		workspace.RunNet(test_model.net)
    		# Collect the batch accuracy from the workspace
    		test_accuracy[i] = workspace.FetchBlob('accuracy')

    	print('Test_accuracy: {:.4f}'.format(test_accuracy.mean()))

    	# == Deployment model. ==
    	# We simply need the main AddModel part.
    	deploy_model = model_helper.ModelHelper(name="deploy_net", arg_scope=arg_scope, init_params=False)
    	self.create_model(deploy_model, "data", device_opts, is_test=True)

    	print("Saving deploy model")
    	self.save_net(self._init_net_, self._predict_net_, deploy_model)

    def save_net(self, init_net_path, predict_net_path, model):

    	init_net, predict_net = mobile_exporter.Export(
    		workspace,
    		model.net,
    		model.params
    	)

        try:
            os.makedirs(self._model_dir_)
        except OSError:
            if not os.path.isdir(self._model_dir_):
                raise

    	print("Save the model to init_net.pb and predict_net.pb")
    	with open(predict_net_path, 'wb') as f:
    		f.write(model.net._net.SerializeToString())
    	with open(init_net_path, 'wb') as f:
    		f.write(init_net.SerializeToString())

    	print("Save the model to init_net.pbtxt and predict_net.pbtxt")

    	with open(init_net_path.replace('.pb','.pbtxt'), 'w') as f:
    		f.write(str(init_net))
    	with open(predict_net_path.replace('.pb','.pbtxt'), 'w') as f:
    		f.write(str(predict_net))
    	print("== Saved init_net and predict_net ==")

    def load_net(self, init_net_path, predict_net_path, device_opts):
        if not os.path.isfile(init_net_path):
            logging.error("Network loading failure. File '" + os.path.abspath(init_net_path) + "' does not exist.")
            sys.exit(1)
        elif not os.path.isfile(predict_net_path):
            logging.error("Network loading failure. File '" + os.path.abspath(predict_net_path) + "' does not exist.")
            sys.exit(1)

        init_def = caffe2_pb2.NetDef()
    	with open(init_net_path, 'rb') as f:
    		init_def.ParseFromString(f.read())
    		init_def.device_option.CopyFrom(device_opts)
    		workspace.RunNetOnce(init_def.SerializeToString())

    	net_def = caffe2_pb2.NetDef()
    	with open(predict_net_path, 'rb') as f:
    		net_def.ParseFromString(f.read())
    		net_def.device_option.CopyFrom(device_opts)
    		workspace.CreateNet(net_def.SerializeToString(), overwrite=True)
    	print("== Loaded init_net and predict_net ==")
