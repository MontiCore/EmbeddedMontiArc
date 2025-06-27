<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
<#setting number_format="computer">
from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2

import numpy as np
import logging
<#list configurations as config>
import CNNCreator_${config.instanceName}
import CNNDataCleaner_${config.instanceName}
</#list>

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

<#list configurations as config>
    ${config.instanceName}_cleaner = CNNDataCleaner_${config.instanceName}.CNNDataCleaner_${config.instanceName}()
    ${config.instanceName} = CNNCreator_${config.instanceName}.CNNCreator_${config.instanceName}(
        ${config.instanceName}_cleaner
    )
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
    <#if (config.cleaning)??>
        cleaning='${config.cleaningName}',
        cleaning_params={
        <#if (config.cleaningParameters)??>
        <#list config.cleaningParameters?keys as param>
            '${param}': ${config.cleaningParameters[param]}<#sep>,
        </#list>
        </#if>
        },
    </#if>
    <#if (config.dataImbalance)??>
        data_imbalance='${config.dataImbalanceName}',
        data_imbalance_params={
        <#if (config.dataImbalanceParameters)??>
        <#list config.dataImbalanceParameters?keys as param>
            '${param}': ${config.dataImbalanceParameters[param]}<#sep>,
        </#list>
        </#if>
        },
    </#if>
    <#if (config.evalMetric)??>
        eval_metric='${config.evalMetricName}',
    </#if>
    <#if (config.loss)??>
        loss='${config.lossName}',
    </#if>
    <#if (config.optimizer)??>
        opt_type='${config.optimizerName}'<#if config.optimizerParameters?has_content>,
        <#list config.optimizerParameters?keys as param>
        <#--To adapt parameter names to Caffe2 since they are different than in CNNTrainLang-->
        <#assign paramName = param>
        <#assign paramValue = config.optimizerParameters[param]>
        <#if param == "learning_rate">
            <#assign paramName = "base_learning_rate">
        <#elseif param == "learning_rate_policy">
            <#assign paramName = "policy">
            <#assign paramValue = "'${paramValue}'">
        <#elseif param == "step_size">
            <#assign paramName = "stepsize">
        <#elseif param == "gamma1">
            <#assign paramName = "gamma1">
        <#elseif param == "learning_rate_decay">
            <#assign paramName = "gamma">
        </#if>
            ${paramName}=${paramValue}<#sep>,
        </#list>
    </#if>
    </#if>

    )
</#list>