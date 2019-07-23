import logging
import os
import errno
import shutil
import h5py
import sys
import numpy as np

import tensorflow as tf



def huber_loss(y_true, y_pred):
    return tf.losses.huber_loss(y_true, y_pred)

def epe(y_true, y_pred):
    return tf.keras.backend.mean(tf.keras.backend.sqrt(tf.keras.backend.sum(tf.keras.backend.square(y_pred - y_true), axis=[3])),axis=[1,2])


def rmse(y_true, y_pred):
    return tf.keras.backend.sqrt(keras.losses.mean_squared_error(y_true, y_pred))    

def f1(y_true, y_pred):
    frac1 = tf.metrics.precision(y_true, y_pred) * tf.metrics.recall(y_true, y_pred)
    frac2 = tf.metrics.precision(y_true, y_pred) + tf.metrics.recall(y_true, y_pred)

    return 2 * frac1 / frac2


class LRScheduler:

    def __init__(self, params):

        self._decay = params["lr_decay"]

        if "lr_policy" in params:
            self._policy = params["lr_policy"]
            print("\n\nUsing %s learning_rate_policy!\n\n", self._policy)
        else:
            self._policy = "fixed"
            print("\n\nNot lerning_rate_policy specified. Using fixed (constant) learning rate!\n\n")

        if "lr_minimum" in params:
            self._minimum = params["lr_minimum"]
        else:
            self._minimum = 1e-08

        if "step_size" in params:    
            self._step_size = params["step_size"]
        else:
            self._step_size = None


        self.scheduler = self.get_lr_scheduler()


    def get_lr_scheduler(self):

        mapping = {
            "fixed":        self.fixed_scheduler,
            "step":         self.step_scheduler}

        mapping_not_supported = {
            "exp":          "exp",
            "inv":          "inv",
            "poly":         "poly",
            "sigmoid":      "sigmoid"}

        if self._policy in mapping:
            return mapping[self._policy]
        elif self._policy in mapping_not_supported:
            #This is due to some parameters neccessery for this policies, missing in the CNNTrainLang grammar, and as the MXNET generator also only implements 
            #the step policy we chose to not add it for now. These policies can be added by using the respective commented out functions below
            logging.warning("The %s learning_rate_policy is currently not supported by the keras/tensorlfow generator. \n", self._policy)
        else:
            logging.warning("The following learning_rate_policy is not supported by the keras/tensorflow generator: %s \n", self._policy)


    #note that the keras callback for lr scheduling only gets called inbetween epochs, not single iterations

    def fixed_scheduler(self, epoch_ind, old_lr):
        return old_lr

    def step_scheduler(self, epoch_ind, old_lr):

        if epoch_ind % self._step_size == 0:
            new_lr = old_lr * self._decay

            if new_lr < self._minimum:
                new_lr = self._minimum
        
            return new_lr
        
        else:
            return old_lr

    #def exp_scheduler(self, epoch_ind, old_lr):
        #return old_lr

    #def inv_scheduler(self, epoch_ind, old_lr):
        #return old_lr

    #def poly_scheduler(self, epoch_ind, old_lr):
        #return old_lr

    #def sigmoid_scheduler(self, epoch_ind, old_lr):
        #return old_lr

    
#If clip weights for rmsProp optimizer is specified this class is needed, as the keras/ tensorflow variant of rmsProp does not support weight clipping
class WeightClip(tf.keras.constraints.Constraint):
    def __init__(self, clip_val=2):
        self.clip_val = clip_val

    def __call__(self, w):
        return K.clip(w, -self.clip_val, self.clip_val)

    def get_config(self):
        return {'name': self.__class__.__name__,
                'clip_val': self.clip_val}

    
    
class CNNCreator_endtoend_nvidia:

    module = None
    _data_dir_ = "resources/training_data/"
    _model_dir_ = "model/endtoend.Nvidia/"
    _model_prefix_ = "model"
    _input_names_ = ['data']
    _output_names_ = ['ergebnis_label']
    _weight_constraint_ = None
    _regularizer_ = None
    _permute_ = False 
     
                      
    def load(self):
        lastEpoch = 0
        model_file = None

        try:
            os.remove(self._model_dir_ + self._model_prefix_ + "_newest.h5py")
        except OSError:
            pass

        if os.path.isdir(self._model_dir_):
            for file in os.listdir(self._model_dir_):
                if ".h5py" in file and self._model_prefix_ in file:
                    epochStr = file.replace(".h5py", "").replace(self._model_prefix_ + ".", "")
                    epoch = int(epochStr)
                    if epoch > lastEpoch:
                        lastEpoch = epoch
                        model_file = file
        if model_file is None:
            return 0
        else:
            logging.info("Loading checkpoint: " + model_file)
            
            self.module = tf.keras.models.load_model(self._model_dir_ + model_file)      
                      
            return lastEpoch

    def load_data_generators(self, batch_size, normalize):
        train_h5, test_h5 = self.load_h5_files()
        
        def get_mean_std(h5_container): # TODO: Calculate this efficiently
            input_means = []
            input_stds = []
            output_means = []
            output_stds = []

            if "statistics" in h5_container.keys():
                logging.info("Statistics found.")
                data = h5_container["statistics"]

                for cur in self._input_names_:
                    input_means.append(data[cur+"_mean"])
                    input_stds.append(data[cur+"_std"])

                for cur in self._output_names_:
                    output_means.append(data[cur+"_mean"])
                    output_stds.append(data[cur+"_std"])
            else:
                logging.info("Statistics NOT found.")
                logging.info("Calculating statistics...")
                
                data_size = h5_container[self._input_names_[0]].shape[0]
                
                logging.info("Calculating statistics for " + str(len(self._input_names_)) + " inputs, " + str(len(self._output_names_)) + " outputs, " + str(data_size) + " data points...")
                
                batch_size = 100

                #Calculating means...
                logging.info("Calculating means... ")

                input_means_running = np.array([0.]*len(self._input_names_))
                output_means_running = np.array([0.]*len(self._output_names_))

                for i in range(0, data_size, batch_size):

                    if i+batch_size < data_size:
                        offset = batch_size
                    else:
                        offset = data_size - i - 1

                    for key in self._input_names_:
                        input_means_running += np.array([np.mean(h5_container[key][i:i+offset].astype(np.float32))*(offset/float(batch_size)) for key in self._input_names_])
                    for key in self._output_names_:
                        output_means_running += np.array([np.mean(h5_container[key][i:i+offset].astype(np.float32))*(offset/float(batch_size)) for key in self._output_names_])

                input_means = list(input_means_running / (data_size//batch_size + offset/float(batch_size)))
                output_means = list(output_means_running / (data_size//batch_size + offset/float(batch_size)))

                logging.info("Done calculating means!")
                logging.info("Input means: " + str(input_means))
                logging.info("Output means: " + str(output_means))


                #Calculating stds...
                logging.info("Calculating stds...")

                input_stds_running = np.array([0.]*len(self._input_names_))
                output_stds_running = np.array([0.]*len(self._output_names_))

                for i in range(0, data_size, batch_size):

                    if i+batch_size < data_size:
                        offset = batch_size
                    else:
                        offset = data_size - i - 1

                    for key in self._input_names_:
                        input_stds_running += np.array([np.mean((h5_container[key][i:i+offset].astype(np.float32) - input_means[index])**2)*(offset/float(batch_size)) for index,key in enumerate(self._input_names_)])
                    for key in self._output_names_:
                        output_stds_running += np.array([np.mean((h5_container[key][i:i+offset].astype(np.float32) - output_means[index])**2)*(offset/float(batch_size)) for index,key in enumerate(self._output_names_)])
                
                std_eps = 1e-08
                input_stds = list(np.sqrt(input_stds_running / (data_size//batch_size + offset/float(batch_size)) + std_eps))
                output_stds = list(np.sqrt(output_stds_running / (data_size//batch_size + offset/float(batch_size)) + std_eps))

                logging.info("Done calculating stds!")
                logging.info("Input stds: " + str(input_stds))
                logging.info("Output stds: " + str(output_stds))

            return input_means, input_stds

        def generator(h5_container, batch_size):
            #all tables in the file have to have same size
            data_size = h5_container[self._input_names_[0]].shape[0]

            while(True):
                for i in range(0, data_size, batch_size):

                    if i+batch_size < data_size:
                        offset = batch_size
                    else:
                        offset = data_size - i - 1

                    if self._permute_ == True:
                        data_batch = [np.transpose(h5_container[key][i:i+offset].astype(np.float32), axes=[0,2,3,1]) for key in self._input_names_]
                        target_batch = [h5_container[key][i:i+offset].astype(np.float32) for key in self._output_names_]
                    else:
                        data_batch = [h5_container[key][i:i+offset].astype(np.float32) for key in self._input_names_]
                        target_batch = [h5_container[key][i:i+offset].astype(np.float32) for key in self._output_names_]

                    yield (data_batch, target_batch)

        train_generator = generator(train_h5, batch_size)
        test_generator = generator(test_h5, batch_size)

        if normalize:
            data_mean, data_std = get_mean_std(train_h5)
        else:
            data_mean, data_std = 0,1

        data_size = train_h5[self._input_names_[0]].shape[0]
        steps_per_epoch = data_size//batch_size
	
        data_size = test_h5[self._input_names_[0]].shape[0]
        validation_steps  = data_size//batch_size

        if validation_steps == 0 and data_size > 0:
            validation_steps = data_size
        elif data_size == 0:
            raise Exception("Test data set is empty. \n")

        return train_generator, test_generator, data_mean, data_std, steps_per_epoch, validation_steps 

    def load_h5_files(self):
        train_h5 = None
        test_h5 = None
        train_path = self._data_dir_ + "train.h5"
        test_path = self._data_dir_ + "test.h5"
        if os.path.isfile(train_path):
            train_h5 = h5py.File(train_path, 'r')
            if not (self._input_names_[0] in train_h5 and self._output_names_[0] in train_h5):
                logging.error("The HDF5 file '" + os.path.abspath(train_path) + "' has to contain the datasets: "
                              + "'" + str(self._input_names_) + "', '" + str(self._output_names_) + "'")
                sys.exit(1)
					  
            if os.path.isfile(test_path):
                test_h5 = h5py.File(test_path, 'r')
                if not (self._input_names_[0] in test_h5 and self._output_names_[0] in test_h5):
                    logging.error("The HDF5 file '" + os.path.abspath(test_path) + "' has to contain the datasets: "
                                  + "'" + str(self._input_names_) + "', '" + str(self._output_names_) + "'")
                    sys.exit(1)
            else:
                logging.warning("Couldn't load test set. File '" + os.path.abspath(test_path) + "' does not exist.")
            return train_h5, test_h5
        else:
            logging.error("Data loading failure. File '" + os.path.abspath(train_path) + "' does not exist.")
            sys.exit(1)

    def build_optimizer(self, optimizer_name, params):
                  
        fixed_params, lr_scheduler_params = self.translate_optimizer_param_names(params)
        
        if optimizer_name == "adam":
            return tf.keras.optimizers.Adam(**fixed_params), lr_scheduler_params
        elif optimizer_name == "sgd":
            return tf.keras.optimizers.SGD(nesterov=False, **fixed_params), lr_scheduler_params
        elif optimizer_name == "nag":
            return tf.keras.optimizers.SGD(nesterov=True, **fixed_params), lr_scheduler_params
        elif optimizer_name == "rmsprop":
            return tf.keras.optimizers.RMSProp(**fixed_params), lr_scheduler_params
        elif optimizer_name == "adagrad":
            return tf.keras.optimizers.Adagrad(**fixed_params), lr_scheduler_params
        elif optimizer_name == "adadelta":
            return tf.keras.optimizers.Adadelta(**fixed_params), lr_scheduler_params
        else:
            logging.warning("Optimizer not supported by keras/tensorflow: %s \n", optimizer_name)

    def translate_optimizer_param_names(self, params):

        mapping = {
            "learning_rate":            "lr",
            "momentum":                 "momentum",
            "beta1":                    "beta_1",
            "beta2":                    "beta_2",
            "gamma1":                   "rho",
            "gamma2":                   "momentum",
            "centered":                 "centered",
            "epsilon":                  "epsilon",
            "rho":                      "rho",
            "clip_gradient":            "clipvalue"}
            
        mapping_lr_scheduler = {
            "learning_rate_decay":      "lr_decay",
            "learning_rate_policy":     "lr_policy",
            "learning_rate_minimum":    "lr_minimum",
            "step_size":                "step_size"}
           
        fixed_params = {}
        lr_scheduler_params = {}
        for k in params:
            if k == "clip_weights":
                self._weight_constraint_ = WeightClip(params[k])
            elif k == "weight_decay":
                self._regularizer_ = tf.keras.regularizers.l2(params[k])
            elif k in mapping_lr_scheduler:
                lr_scheduler_params[mapping_lr_scheduler[k]] = params[k]
            elif k in mapping.keys():
                fixed_params[mapping[k]] = params[k]
            else:
                logging.warning("The following parameter is not supported by the keras/tensorflow generator %s \n", k)          
        return fixed_params, lr_scheduler_params
    
    
    def translate_loss_names(self, losses, num_outputs):
        mapping = {
            "l2":                   "mean_squared_error",
            "l1":                   "mean_absolute_error",
            "cross_entropy":        "sparse_categorical_crossentropy" if num_outputs > 1 else "binary_crossentropy",
            "log_cosh":             "logcosh",
            "hinge":                "hinge",
            "squared_hinge":        "squared_hinge",
            "kullback_leibler":     "kullback_leibler_divergence",
            "huber_loss":           huber_loss,
            "epe":                  epe}
                      
        fixed_loss_names = []
        for k in losses:
            if k in mapping.keys():
                fixed_loss_names.append(mapping[k])
            else:
                logging.warning("The following loss is not supported by the keras/tensorflow generator:%s \n", k)
        return fixed_loss_names
				                           
    def translate_eval_metric_names(self, metrics, num_outputs):
        mapping = {
            "accuracy":             "acc",
            "mse":                  "mse",
            "mae":                  "mae",
            "rmse":                 rmse,
            "top_k_accuracy":       "top_k_categorical_accuracy",
            "cross_entropy":        "sparse_categorical_crossentropy" if num_outputs > 1 else "binary_crossentropy",
            "f1":                   f1}

        fixed_metric_names = []
        for k in metrics:
            if k in mapping.keys():
                fixed_metric_names.append(mapping[k])
            elif k != []:
                logging.warning("The following metric is not supported by the keras/tensorflow generator: %s \n", k)
        return fixed_metric_names

    def train(self, batch_size=64,
              num_epoch=10,
              eval_metric=[],
			  loss="cross_entropy",
              optimizer='adam',
              optimizer_params=(('learning_rate', 0.001),),
              load_checkpoint=True,
              context='gpu',
              checkpoint_period=5,
              normalize=True):

        if context=="cpu":
            print("\n\n\nExecution in CPU mode.\n\n\n")
            os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
        else:
            print("\n\n\nExecution in GPU mode.\n\n\n")
                      
        
        train_gen, test_gen, data_mean, data_std, steps_per_epoch, validation_steps = self.load_data_generators(batch_size, normalize)

        if self.module == None:
            if normalize:
                self.construct(data_mean, data_std)
            else:
                self.construct()
                
        
        optimizer_instance, lr_scheduler_params = self.build_optimizer(optimizer, optimizer_params)
        
        
        num_outputs = self.module.layers[-1].output_shape[1]
        metrics = self.translate_eval_metric_names([eval_metric], num_outputs)
        tf_loss = self.translate_loss_names([loss], num_outputs)
        

        begin_epoch = 0
        if load_checkpoint:
            begin_epoch = self.load()
        else:
            if os.path.isdir(self._model_dir_):
                shutil.rmtree(self._model_dir_)

            self.module.compile(
                optimizer=optimizer_instance,
                loss=tf_loss,
                metrics=metrics)

        try:
            os.makedirs(self._model_dir_)
        except OSError:
            if not os.path.isdir(self._model_dir_):
                raise

                
        #callbacks
        model_checkpoints_cb = tf.keras.callbacks.ModelCheckpoint(filepath=self._model_dir_ + self._model_prefix_ + "." + "{epoch:d}.hdf5", verbose=0, save_weights_only=False, period=checkpoint_period)
            
        if "lr_decay" in lr_scheduler_params:
            lr_scheduler = LRScheduler(lr_scheduler_params)
            lr_scheduler_cb = tf.keras.callbacks.LearningRateScheduler(lr_scheduler.scheduler, 1)
            
            callbacks = [model_checkpoints_cb, lr_scheduler_cb]
        else:
            callbacks = [model_checkpoints_cb]
            
            
        
        self.module.fit_generator(
            generator=train_gen,
            validation_data=test_gen,
            epochs=num_epoch,
	        validation_steps = validation_steps,
            steps_per_epoch = steps_per_epoch,
			callbacks=callbacks) 
        

        #saving curent state of model as .h5py for resuming training in python
        tf.keras.models.save_model(self.module, self._model_dir_ + self._model_prefix_ + "." + str(num_epoch + begin_epoch) + ".hdf5")
        tf.keras.models.save_model(self.module, self._model_dir_ + self._model_prefix_ + ".newest.hdf5")

        #Saving model in .pb format for prediction in c++
        saver = tf.train.Saver()
        sess = tf.keras.backend.get_session()
        save_path = saver.save(sess, self._model_dir_ + self._model_prefix_ + "_cpp_pred")


    def construct(self, data_mean=None, data_std=None):
	
        input_tensors = []
        output_names = []
	
        
               
        #We Want channels last for tensorflow
        self._permute_ = True
        
        

        data = tf.keras.layers.Input(shape=(256,455,9), name="data")
        input_tensors.append(data)
        # data, output shape: {[256,455,9]}

            

        #TODO: Test this
        if not data_mean is None:
            assert(not data_std is None)

            data  = tf.keras.layers.Lambda(lambda x : (x - data_mean)/data_std)(data)

            data = tf.keras.layers.Lambda(lambda x: tf.keras.backend.stop_gradient(x))(data)
	
        batchnorm1_ = tf.keras.layers.BatchNormalization(beta_regularizer=self._regularizer_,
                                                             gamma_regularizer=self._regularizer_,
                                                             beta_constraint=self._weight_constraint_,
                                                             gamma_constraint=self._weight_constraint_,
                                                             name="batchnorm1_",)(data) #TODO: fix_gamma=True

        #TODO check for padding
        conv1_ = tf.keras.layers.Conv2D(3, 
                                                 kernel_size=(5,5), 
                                                 strides=(1,1), 
                                                 padding="same", 
                                                 use_bias=True, 
                                                 kernel_regularizer=self._regularizer_, 
                                                 kernel_constraint=self._weight_constraint_, 
                                                 name="conv1_")(batchnorm1_)
        # conv1_, output shape: {[256,455,3]}

        relu1_ = tf.keras.layers.Activation(activation = "relu", name="relu1_")(conv1_)

        #TODO check for padding
        conv2_ = tf.keras.layers.Conv2D(3, 
                                                 kernel_size=(5,5), 
                                                 strides=(1,1), 
                                                 padding="same", 
                                                 use_bias=True, 
                                                 kernel_regularizer=self._regularizer_, 
                                                 kernel_constraint=self._weight_constraint_, 
                                                 name="conv2_")(relu1_)
        # conv2_, output shape: {[256,455,3]}

        relu2_ = tf.keras.layers.Activation(activation = "relu", name="relu2_")(conv2_)

        #TODO check for padding
        conv3_ = tf.keras.layers.Conv2D(3, 
                                                 kernel_size=(3,3), 
                                                 strides=(1,1), 
                                                 padding="same", 
                                                 use_bias=True, 
                                                 kernel_regularizer=self._regularizer_, 
                                                 kernel_constraint=self._weight_constraint_, 
                                                 name="conv3_")(relu2_)
        # conv3_, output shape: {[256,455,3]}

        relu3_ = tf.keras.layers.Activation(activation = "relu", name="relu3_")(conv3_)

        #TODO check for padding
        conv4_ = tf.keras.layers.Conv2D(3, 
                                                 kernel_size=(3,3), 
                                                 strides=(1,1), 
                                                 padding="same", 
                                                 use_bias=True, 
                                                 kernel_regularizer=self._regularizer_, 
                                                 kernel_constraint=self._weight_constraint_, 
                                                 name="conv4_")(relu3_)
        # conv4_, output shape: {[256,455,3]}

        relu4_ = tf.keras.layers.Activation(activation = "relu", name="relu4_")(conv4_)

        dropout4_ = tf.keras.layers.Dropout(rate = 0.5, name="dropout4_")(relu4_)

    
        pool4_ = tf.keras.layers.MaxPool2D(pool_size = (2,2), #or element.poolsize?
            strides = (1,1), #or element.strides?  (plural)
            padding = "same",
            data_format = "channels_last",
            name="pool4_")(dropout4_)
        # pool4_, output shape: {[256,455,3]}

        flatten4_ = tf.keras.layers.Flatten()(pool4_)


        fc4_ = tf.keras.layers.Dense(100,
                                                use_bias=False,
                                                kernel_regularizer=self._regularizer_, 
                                                kernel_constraint=self._weight_constraint_, 
                                                name="fc4_")(flatten4_)

        relu5_ = tf.keras.layers.Activation(activation = "relu", name="relu5_")(fc4_)

        dropout5_ = tf.keras.layers.Dropout(rate = 0.5, name="dropout5_")(relu5_)


        fc5_ = tf.keras.layers.Dense(50,
                                                use_bias=False,
                                                kernel_regularizer=self._regularizer_, 
                                                kernel_constraint=self._weight_constraint_, 
                                                name="fc5_")(dropout5_)

        relu6_ = tf.keras.layers.Activation(activation = "relu", name="relu6_")(fc5_)

        dropout6_ = tf.keras.layers.Dropout(rate = 0.5, name="dropout6_")(relu6_)


        fc6_ = tf.keras.layers.Dense(10,
                                                use_bias=False,
                                                kernel_regularizer=self._regularizer_, 
                                                kernel_constraint=self._weight_constraint_, 
                                                name="fc6_")(dropout6_)

        relu7_ = tf.keras.layers.Activation(activation = "relu", name="relu7_")(fc6_)

        dropout7_ = tf.keras.layers.Dropout(rate = 0.5, name="dropout7_")(relu7_)


        fc7_ = tf.keras.layers.Dense(1,
                                                use_bias=False,
                                                kernel_regularizer=self._regularizer_, 
                                                kernel_constraint=self._weight_constraint_, 
                                                name="fc7_")(dropout7_)

        relu8_ = tf.keras.layers.Activation(activation = "relu", name="relu8_")(fc7_)

        dropout8_ = tf.keras.layers.Dropout(rate = 0.5, name="dropout8_")(relu8_)

        sigmoid8_ = tf.keras.layers.Activation("sigmoid", name="sigmoid8_")(dropout8_)
        #Just an "Identity" layer with the appropriate output name, as if softmax is required it is generated from the Softmax.ftl template        
        ergebnis = tf.keras.layers.Lambda(lambda x: x, name="ergebnis")(sigmoid8_)
        output_names.append("ergebnis")
            
        
        self.module = tf.keras.models.Model(inputs=input_tensors, outputs=[ergebnis])
             
				
        out_nodes = [node for node in self.module.outputs] 
        tf.identity(out_nodes[0], name="output_0")
