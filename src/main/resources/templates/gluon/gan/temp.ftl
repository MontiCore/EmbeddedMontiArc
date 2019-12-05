<#setting number_format="computer">
<#assign config = configurations[0]>
<#assign rlAgentType=config.rlAlgorithm?switch("dqn", "DqnAgent", "ddpg", "DdpgAgent", "td3", "TwinDelayedDdpgAgent")>
from ${ganFrameworkModule}.CNNCreator_${discriminatorInstanceName} import CNNCreator_${discriminatorInstanceName}
import CNNCreator_${config.instanceName}

import mxnet as mx
import logging
import numpy as np
import time
import os
import shutil
from mxnet import gluon, autograd, nd

if __name__ = "__main__":

    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

<#if (config.context)??>
    context = mx.${config.context}()
<#else>
    context = mx.cpu()
</#if>

    generator_creator = CNNCreator_${config.instanceName}.CNNCreator_${config.instanceName}()
    generator_creator.construct(context)
    discriminator_creator = CNNCreator_${discriminatorInstanceName}()
    discriminator_creator.construct(context)

<#if (config.batchSize)??>
    batch_size=${config.batchSize},
</#if>

<#if (config.numEpoch)??>
    num_epoch=${config.numEpoch},
</#if>

<#if (config.loadCheckpoint)??>
    load_checkpoint=${config.loadCheckpoint?string("True","False")},
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

<#if (config.configuration.optimizer)??>
    optimizer='${config.optimizerName}',
    optimizer_params={
<#list config.optimizerParams?keys as param>
        '${param}': ${config.optimizerParams[param]}<#sep>,
</#list>
    }
</#if>

<#if (config.noiseDistribution)??>
    noise_distribution = '${config.noiseDistribution.name}',
    noise_distribution_params = {
<#if (config.noiseDistribution.mean_value)??>
        'mean_value': ${config.noiseDistribution.mean_value},
</#if>
<#if (config.noiseDistribution.spread_value??>
        'spread_value: ${config.noiseDistribution.spread_value}
</#if>
     }
</#if>
    print(noise_distribution_params)
    print(noise_distribution)
