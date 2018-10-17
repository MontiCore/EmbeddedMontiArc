from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2

import numpy as np
import cv2
import logging
<#list configurations as config>
import CNNCreator_${config.instanceName}
</#list>

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

<#list configurations as config>
    ${config.instanceName} = CNNCreator_${config.instanceName}.CNNCreator_${config.instanceName}()
    ${config.instanceName}.train(
<#if (config.numEpoch)??>
        num_epoch=${config.numEpoch},
</#if>
<#if (config.batchSize)??>
        batch_size=${config.batchSize},
</#if>
<#if (config.context)??>
        context='${config.context}',
</#if>
<#if (config.evalMetric)??>
        eval_metric='${config.evalMetric}',
</#if>
<#if (config.configuration.optimizer)??>
        opt_type='${config.optimizerName}',
<#list config.optimizerParams?keys as param>
    <#--Adapt parameter names since parameter names in Caffe2 are different than in CNNTrainLang-->
    <#assign paramName = param>
    <#if param == "learning_rate">
        <#assign paramName = "base_learning_rate">
    <#elseif param == "learning_rate_policy">
        <#assign paramName = "policy">
    <#elseif param == "step_size">
        <#assign paramName = "stepsize">
    <#elseif param == "gamma1">
        <#assign paramName = "gamma">
    </#if>
        ${paramName}=${config.optimizerParams[param]}<#sep>,
</#list>
</#if>

    )
</#list>

<#--Below code can be removed. It is only an specific example to verify that deploy_net works-->
    print '\n********************************************'
    print("Loading Deploy model")

<#list configurations as config>
<#if (config.context)??>
    context='${config.context}'
<#else>
    context = 'gpu'
</#if>
<#--Code section that decides the mode cannot be moved into the load_net function since workspace.FeedBlob also needs this parameter-->
    if context == 'cpu':
        device_opts = core.DeviceOption(caffe2_pb2.CPU, 0)
        print("CPU mode selected")
    elif context == 'gpu':
        device_opts = core.DeviceOption(caffe2_pb2.CUDA, 0)
        print("GPU mode selected")
</#list>

    LeNet.load_net(LeNet.INIT_NET, LeNet.PREDICT_NET, device_opts=device_opts)

    img = cv2.imread("3.jpg")                                   # Load test image
    img = cv2.resize(img, (28,28))                              # Resize to 28x28
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY )                # Covert to grayscale
    img = img.reshape((1,1,28,28)).astype('float32')            # Reshape to (1,1,28,28)
    workspace.FeedBlob("data", img, device_option=device_opts)  # FeedBlob
    workspace.RunNet('deploy_net', num_iter=1)                  # Forward

    print("\nInput: {}".format(img.shape))
    pred = workspace.FetchBlob("predictions")
    print("Output: {}".format(pred))
    print("Output class: {}".format(np.argmax(pred)))
