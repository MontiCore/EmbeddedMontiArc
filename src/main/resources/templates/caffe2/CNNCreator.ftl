from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2
import numpy as np

#import logging
import os
#import shutil
#import sys
#import cv2

class ${tc.fileNameWithoutEnding}:

    module = None
    _data_dir_ = "data/${tc.fullArchitectureName}/"
    _model_dir_ = "model/${tc.fullArchitectureName}/"
    _model_prefix_ = "${tc.architectureName}"
    _input_names_ = [${tc.join(tc.architectureInputs, ",", "'", "'")}]
    _input_shapes_ = [<#list tc.architecture.inputs as input>(${tc.join(input.definition.type.dimensions, ",")})</#list>]
    _output_names_ = [${tc.join(tc.architectureOutputs, ",", "'", "_label'")}]


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

    def add_input(self, model, batch_size, db, db_type, device_opts):
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

    def create_model(self, model, data, device_opts):
    	with core.DeviceScope(device_opts):

${tc.include(tc.architecture.body)}

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

    def train(self, num_epoch=1000, batch_size=64, device_opts='gpu', eval_metric='accuracy', opt_type='adam', base_learning_rate=0.001, weight_decay=0.001, policy='fixed', stepsize=1, epsilon=1E-8, beta1=0.9, beta2=0.999, gamma=0.999, momentum=0.9) :
        if device_opts == 'cpu':
            device_opts = core.DeviceOption(caffe2_pb2.CPU, 0)
            print("CPU mode selected")
        elif device_opts == 'gpu':
            device_opts = core.DeviceOption(caffe2_pb2.CUDA, 0)
            print("GPU mode selected")

    	workspace.ResetWorkspace(self.ROOT_FOLDER)

    	arg_scope = {"order": "NCHW"}
    	# == Training model ==
    	train_model= model_helper.ModelHelper(name="train_net", arg_scope=arg_scope)
    	data, label = self.add_input(train_model, batch_size=batch_size, db=os.path.join(self.DATA_FOLDER, 'mnist-train-nchw-lmdb'), db_type='lmdb', device_opts=device_opts)
    	${tc.join(tc.architectureOutputs, ",", "","")} = self.create_model(train_model, data, device_opts=device_opts)
    	self.add_training_operators(train_model, ${tc.join(tc.architectureOutputs, ",", "","")}, label, device_opts, opt_type, base_learning_rate, policy, stepsize, epsilon, beta1, beta2, gamma, momentum)
    	self.add_accuracy(train_model, ${tc.join(tc.architectureOutputs, ",", "","")}, label, device_opts, eval_metric)
    	with core.DeviceScope(device_opts):
    		brew.add_weight_decay(train_model, weight_decay)

    	# Initialize and create the training network
    	workspace.RunNetOnce(train_model.param_init_net)
    	workspace.CreateNet(train_model.net, overwrite=True)

    	# Main Training Loop
    	print("== Starting Training for " + str(num_epoch) + " num_epoch ==")
    	for j in range(0, num_epoch):
    		workspace.RunNet(train_model.net)
    		if j % 50 == 0:
    			print 'Iter: ' + str(j) + ': ' + 'Loss ' + str(workspace.FetchBlob("loss")) + ' - ' + 'Accuracy ' + str(workspace.FetchBlob('accuracy'))
    	print("Training done")

    	print("== Running Test model ==")
    	# == Testing model. ==
    	test_model= model_helper.ModelHelper(name="test_net", arg_scope=arg_scope, init_params=False)
    	data, label = self.add_input(test_model, batch_size=100, db=os.path.join(self.DATA_FOLDER, 'mnist-test-nchw-lmdb'), db_type='lmdb', device_opts=device_opts)
    	${tc.join(tc.architectureOutputs, ",", "","")} = self.create_model(test_model, data, device_opts=device_opts)
    	self.add_accuracy(test_model, predictions, label, device_opts, eval_metric)
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
    	self.create_model(deploy_model, "data", device_opts)

    	print("Saving deploy model")
    	self.save_net(self.INIT_NET, self.PREDICT_NET, deploy_model)

    def save_net(self, init_net_path, predict_net_path, model):

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

    def load_net(self, init_net_path, predict_net_path, device_opts):
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
