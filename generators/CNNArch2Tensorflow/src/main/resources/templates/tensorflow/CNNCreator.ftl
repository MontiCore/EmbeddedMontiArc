<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import logging
import os
import errno
import shutil
import h5py
import sys
import numpy as np
import tensorflow as tf
from CNNDataLoader_${tc.fullArchitectureName} import CNNDataLoader_${tc.fullArchitectureName} as CNNDataLoader


def huber_loss(y_true, y_pred):
    return tf.losses.huber_loss(y_true, y_pred)

def epe(y_true, y_pred):
    return tf.keras.backend.mean(tf.keras.backend.sqrt(tf.keras.backend.sum(tf.keras.backend.square(y_pred - y_true), axis=[1])),axis=[1,2])


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
            logging.info("Using %s learning_rate_policy!\n\n", self._policy)
        else:
            self._policy = "step"
            logging.warning("No lerning_rate_policy specified. Using step scheduler!\n\n")

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
            #the step policy at the time of implementing this, we chose to not add it for now. These policies can be added by using the respective commented out functions below
            logging.warning("The %s learning_rate_policy is currently not supported by the keras/tensorlfow generator. \n", self._policy)
        else:
            logging.warning("The following learning_rate_policy is not supported by the keras/tensorflow generator: %s \n", self._policy)


    #note that the keras callback for lr scheduling only gets called inbetween epochs, not single iterations
    def fixed_scheduler(self, epoch_ind, old_lr):
        return old_lr

    def step_scheduler(self, epoch_ind, old_lr):

        if (epoch_ind % self._step_size == 0) and epoch_ind > 0:
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

    
    
class ${tc.fileNameWithoutEnding}:
     
    def __init__(self):
        self.model = None
        self._data_dir_ = "${tc.dataPath}/"
        self._model_dir_ = "model/${tc.componentName}/"
        self._model_prefix_ = "model"
        
        self._input_names_ = [<#list tc.architectureInputs as inputName>'${inputName?keep_before_last("_")}'<#sep>, </#list>]
        self._output_names_ = [${tc.join(tc.architectureOutputs, ",", "'", "label'")}]
        self._output_shapes_ = [<#list tc.architecture.outputs as output>(${tc.join(output.ioDeclaration.type.dimensions, ",")},)<#sep>,</#list>]
        
        self._weight_constraint_ = None
        self._regularizer_ = None
    
    def load(self):
        lastEpoch = 0
        model_file = None

        try:
            os.remove(self._model_dir_ + self._model_prefix_ + ".newest.hdf5")
        except OSError:
            pass

        if os.path.isdir(self._model_dir_):
            for file in os.listdir(self._model_dir_):
                if ".hdf5" in file and self._model_prefix_ in file:
                    epochStr = file.replace(".hdf5", "").replace(self._model_prefix_ + ".", "")
                    epoch = int(epochStr)
                    if epoch > lastEpoch:
                        lastEpoch = epoch
                        model_file = file
        if model_file is None:
            return 0
        else:
            logging.info("Loading checkpoint: " + model_file)

            self.model = tf.keras.models.load_model(self._model_dir_ + model_file)

            return lastEpoch

    def build_optimizer(self, optimizer_name, params):

        fixed_params, lr_scheduler_params = self.translate_optimizer_param_names(params)

        if optimizer_name == "adam":
            return tf.keras.optimizers.Adam(**fixed_params), lr_scheduler_params
        elif optimizer_name == "sgd":
            return tf.keras.optimizers.SGD(nesterov=False, **fixed_params), lr_scheduler_params
        elif optimizer_name == "nag":
            return tf.keras.optimizers.SGD(nesterov=True, **fixed_params), lr_scheduler_params
        elif optimizer_name == "rmsprop":
            return tf.keras.optimizers.RMSprop(**fixed_params), lr_scheduler_params
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

    def translate_loss_name(self, loss, num_outputs):
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

        if loss in mapping.keys():
        	fixed_loss = mapping[loss]
        else:
            logging.warning("The following loss is not supported by the keras/tensorflow generator:%s \n", k)
        return fixed_loss

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
              loss_weights=None,
              optimizer='adam',
              optimizer_params=(('learning_rate', 0.001),),
              load_checkpoint=True,
              context='gpu',
              checkpoint_period=5,
              normalize=True,
              onnx_export=False):
                                
        if context=="cpu":
            os.environ["CUDA_VISIBLE_DEVICES2"] = '-1'

        dataLoader = CNNDataLoader(self._data_dir_, self._input_names_, self._output_names_, self._output_shapes_)
        train_gen, test_gen, data_mean, data_std, steps_per_epoch, validation_steps = dataLoader.load_data_generators(batch_size, normalize)

        if self.model== None:
            if normalize:
                self.construct(data_mean, data_std)
            else:
                self.construct()

        optimizer_instance, lr_scheduler_params = self.build_optimizer(optimizer, optimizer_params)

        num_outputs = self.model.layers[-1].output_shape[1]
        metrics = self.translate_eval_metric_names([eval_metric], num_outputs)
        tf_loss = self.translate_loss_name(loss, num_outputs)

        begin_epoch = 0
        if load_checkpoint:
            begin_epoch = self.load()
            if begin_epoch == 0:
                if os.path.isdir(self._model_dir_):
                    shutil.rmtree(self._model_dir_)

                self.model.compile(
                    optimizer=optimizer_instance,
                    loss=tf_loss,
                    loss_weights=loss_weights,
                    metrics=metrics)     
        else:
            if os.path.isdir(self._model_dir_):
                shutil.rmtree(self._model_dir_)

            self.model.compile(
                optimizer=optimizer_instance,
                loss=tf_loss,
                loss_weights=loss_weights,
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

        self.model.fit_generator(
            generator=train_gen,
            validation_data=test_gen,
            epochs=num_epoch,
	        validation_steps = validation_steps,
            steps_per_epoch = steps_per_epoch,
			callbacks=callbacks) 


        #saving curent state of model as .h5py for resuming training in python
        tf.keras.models.save_model(self.model, self._model_dir_ + self._model_prefix_ + "." + str(num_epoch + begin_epoch) + ".hdf5")
        tf.keras.models.save_model(self.model, self._model_dir_ + self._model_prefix_ + ".newest.hdf5")

        #Saving model in .pb format for prediction in c++
        saver = tf.train.Saver()
        sess = tf.keras.backend.get_session()
        save_path = saver.save(sess, self._model_dir_ + self._model_prefix_ + "_cpp_pred")

        if onnx_export:
            import onnx
            import onnxmltools
            onnx_model = onnxmltools.convert_keras(self.model)
            onnx.save(onnx_model, self._model_dir_ + self._model_prefix_ + ".onnx")

    def construct(self, data_mean=None, data_std=None):
	
        input_tensors = []
        output_names = []

<#list tc.architecture.streams as stream>
<#--<#if stream.isTrainable()>-->
                                
#************* Start Stream ${stream?index}*****************************
${tc.include(stream)}
#************* End Stream ${stream?index}*******************************        
<#--</#if>-->
</#list>           
                 
        self.model = tf.keras.models.Model(inputs=input_tensors, outputs=[${tc.join(tc.architectureOutputs, ",")}])
             
        for i, node in enumerate(self.model.outputs):
            tf.identity(node, name="output_" + str(i))
