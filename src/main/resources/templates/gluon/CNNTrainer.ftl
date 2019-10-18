import logging
import mxnet as mx
<#list configurations as config>
import CNNCreator_${config.instanceName}
import CNNDataLoader_${config.instanceName}
import CNNSupervisedTrainer_${config.instanceName}
</#list>

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

<#list configurations as config>
    ${config.instanceName}_creator = CNNCreator_${config.instanceName}.CNNCreator_${config.instanceName}()
    ${config.instanceName}_loader = CNNDataLoader_${config.instanceName}.CNNDataLoader_${config.instanceName}()
    ${config.instanceName}_trainer = CNNSupervisedTrainer_${config.instanceName}.CNNSupervisedTrainer_${config.instanceName}(
        ${config.instanceName}_loader,
        ${config.instanceName}_creator
    )

    ${config.instanceName}_trainer.train(
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
        eval_metric='${config.evalMetric.metric}',
        eval_metric_params={
<#if (config.evalMetric.exclude)??>
            'exclude': [<#list config.evalMetric.exclude as value>${value}<#sep>, </#list>],
</#if>
        },
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