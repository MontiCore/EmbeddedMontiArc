<#setting number_format="computer">
<#assign config = configurations[0]>

import mxnet as mx
import logging
import os
import numpy as np
import time
import shutil
from mxnet import gluon, autograd, nd

import CNNCreator_${config.instanceName}
import CNNDataLoader_${config.instanceName}
import CNNGanTrainer_${config.instanceName}

from ${ganFrameworkModule}.CNNCreator_${discriminatorInstanceName} import CNNCreator_${discriminatorInstanceName}
<#if (qNetworkInstanceName)??>
from ${ganFrameworkModule}.CNNCreator_${qNetworkInstanceName} import CNNCreator_${qNetworkInstanceName}
</#if>

if __name__ == "__main__":

    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    data_loader = CNNDataLoader_${config.instanceName}.CNNDataLoader_${config.instanceName}()

    gen_creator = CNNCreator_${config.instanceName}.CNNCreator_${config.instanceName}()
    dis_creator = CNNCreator_${discriminatorInstanceName}()
    <#if (qNetworkInstanceName)??>
    qnet_creator = CNNCreator_${qNetworkInstanceName}()
    </#if>

    ${config.instanceName}_trainer = CNNGanTrainer_${config.instanceName}.CNNGanTrainer_${config.instanceName}(
        data_loader,
        gen_creator,
        dis_creator,
        <#if (qNetworkInstanceName)??>
        qnet_creator
        </#if>
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
<#if (config.preprocessingName)??>
        preprocessing=${config.preprocessingName???string("True","False")},
</#if>
<#if (config.imgResizeWidth)??>
<#if (config.imgResizeHeight)??>
        img_resize=(${config.imgResizeWidth}, ${config.imgResizeHeight}),
</#if>
</#if>
<#if (config.evalMetric)??>
        eval_metric='${config.evalMetric}',
</#if>
<#if (config.configuration.loss)??>
        loss ='${config.lossName}',
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
        },
</#if>
<#if (config.noiseDistribution)??>
        noise_distribution = '${config.noiseDistribution.name}',
        noise_distribution_params = {
<#if (config.noiseDistribution.mean_value)??>
            'mean_value': ${config.noiseDistribution.mean_value},
</#if>
<#if (config.noiseDistribution.spread_value)??>
            'spread_value': ${config.noiseDistribution.spread_value}
</#if>
        })
</#if>


