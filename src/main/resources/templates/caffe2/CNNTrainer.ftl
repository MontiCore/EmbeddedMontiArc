from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2

import numpy as np
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
    <#--To adapt parameter names since parameter names in Caffe2 are different than in CNNTrainLang-->
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
