import mxnet as mx
import numpy as np
import h5py
import logging

logging.basicConfig(level=logging.DEBUG)

def create_model():
    input = mx.sym.var("data")
<#list layers as layer>
    ${tc.include(layer)}
</#list>
    model = mx.mod.Module(symbol=${tc.name(outputLayer)}, context=mx.${context?lower_case}())
    return model

def train_model(model):
    mnist = mx.test_utils.get_mnist()
    train_iter = mx.io.NDArrayIter(mnist['train_data'], mnist['train_label'], ${tc.print(conf, "batch_size")})
    val_iter = mx.io.NDArrayIter(mnist['test_data'], mnist['test_label'], ${tc.print(conf, "batch_size")})
    #file = h5py.File("")
    #g = file.require_group("train")
    #train_iter = mx.io.NDArrayIter(g.require_dataset("data"), g.require_dataset("labels"), )
    #g = file.require_group("test")
    #val_iter = mx.io.NDArrayIter(g.require_dataset("data"), g.require_dataset("labels"),)

    model.fit(
        train_data=train_iter,
        eval_data=val_iter,
        optimizer=${tc.print(conf, "optimizer")},
        optimizer_params={'learning_rate':${tc.print(conf, "learning_rate")}},
        eval_metric=${tc.print(conf, "eval_metric")!"\"acc\""},
        batch_end_callback = mx.callback.Speedometer(${tc.print(conf, "batch_size")}, ${tc.print(conf, "log_frequency")!"100"}),
        num_epoch=${tc.print(conf, "epochs")})

model = None

if __name__ == "__main__":
    if (model == None):
        model = create_model()
        train_model(model)
