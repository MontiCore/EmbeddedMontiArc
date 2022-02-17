<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
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
<#if (config.batchSize)??>
        batch_size=${config.batchSize},
</#if>
<#if (config.numEpoch)??>
        num_epoch=${config.numEpoch},
</#if>
<#if (config.loadCheckpoint)??>
        load_checkpoint=${config.loadCheckpoint?string("True","False")},
</#if>
<#if (config.context)??>
        context='${config.context}',
</#if>
<#if (config.normalize)??>
        normalize=${config.normalize?string("True","False")},
</#if>
<#if (config.evalMetric)??>
        eval_metric='${config.evalMetricName}',
</#if>
<#if (config.loss)??>
        loss='${config.lossName}',
<#if (config.lossParameters)??>
        loss_params={
<#list config.lossParameters?keys as param>
            '${param}': ${config.lossParameters[param]}<#sep>,
</#list>
},
</#if>
</#if>
<#if (config.lossWeights)??>
        loss_weights=${config.lossWeights},
</#if>        
<#if (config.optimizer)??>
        optimizer='${config.optimizerName}',
        optimizer_params={
    <#list config.optimizerParameters?keys as param>
        <#assign paramName = param>
        <#assign paramValue = config.optimizerParameters[param]>
        <#if param == "learning_rate_policy">
            <#assign paramValue = "'${paramValue}'">
        </#if>
        '${paramName}': ${paramValue}<#sep>,
    </#list>
},
</#if>
<#if (config.onnxExport)??>
        onnx_export=${config.onnxExport?string("True","False")},
</#if>
    )
</#list>