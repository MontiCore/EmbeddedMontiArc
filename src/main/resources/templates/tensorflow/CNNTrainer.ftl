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
        eval_metric='${config.evalMetric}',
</#if>
<#if (config.configuration.loss)??>
        loss='${config.lossName}',
<#if (config.lossParams)??>
        loss_params={
<#list config.lossParams?keys as param>
            '${param}': ${config.lossParams[param]}<#sep>,
</#list>
},
</#if>
</#if>
<#if (config.lossWeights)??>
        loss_weights=${config.lossWeights},
</#if>        
<#if (config.configuration.optimizer)??>
        optimizer='${config.optimizerName}',
        optimizer_params={
<#list config.optimizerParams?keys as param>
            '${param}': ${config.optimizerParams[param]}<#sep>,
</#list>
}
</#if>
    )
</#list>