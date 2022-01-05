<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import logging
import mxnet as mx
import sys
sys.path.append('./gnn')
sys.path.append('./target/gnn')

<#list configurations as config>
import CNNGnnCreator_${config.instanceName}
import CNNGnnDataLoader_${config.instanceName}
import CNNGnnSupervisedTrainer_${config.instanceName}
</#list>

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

<#list configurations as config>
    ${config.instanceName}_creator = CNNGnnCreator_${config.instanceName}.CNNGnnCreator()
    ${config.instanceName}_creator.validate_parameters()
    ${config.instanceName}_loader = CNNGnnDataLoader_${config.instanceName}.CNNGnnDataLoader()
    ${config.instanceName}_trainer = CNNGnnSupervisedTrainer_${config.instanceName}.CNNGnnSupervisedTrainer(
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
    <#if (config.checkpointPeriod)??>
        checkpoint_period=${config.checkpointPeriod},
    </#if>
    <#if (config.logPeriod)??>
        log_period=${config.logPeriod},
    </#if>
    <#if (config.loadPretrained)??>
        load_pretrained=${config.loadPretrained?string("True","False")},
    </#if>
    <#if (config.context)??>
        context='${config.context}',
    </#if>
    <#if (config.preprocessor)??>
        preprocessing=${config.preprocessor?string("True","False")},
    </#if>
    <#if (config.normalize)??>
        normalize=${config.normalize?string("True","False")},
    </#if>
    <#if (config.shuffleData)??>
        shuffle_data=${config.shuffleData?string("True","False")},
    </#if>
    <#if (config.argmaxAxis)??>
        argmax_axis=${config.argmaxAxis},
    </#if>
    <#if (config.useDgl)??>
        use_dgl='${config.useDgl?string("True","False")}',
    </#if>
    <#if (config.clipGlobalGradNorm)??>
        clip_global_grad_norm=${config.clipGlobalGradNorm},
    </#if>
    <#if (config.useTeacherForcing)??>
        use_teacher_forcing='${config.useTeacherForcing?string("True","False")}',
    </#if>
    <#if (config.saveAttentionImage)??>
        save_attention_image='${config.saveAttentionImage?string("True","False")}',
    </#if>
    <#if (config.trainMask)??>
        train_mask=[<#list config.trainMask as t>${t}<#if t?has_next>, </#if></#list>],
    </#if>
    <#if (config.testMask)??>
        test_mask=[<#list config.testMask as t>${t}<#if t?has_next>, </#if></#list>],
    </#if>
    <#if (config.evalMetric)??>
        eval_metric='${config.evalMetricName}',
        eval_metric_params={
        <#if (config.evalMetricParameters)??>
        <#list config.evalMetricParameters?keys as param>
            '${param}': ${config.evalMetricParameters[param]}<#sep>,
        </#list>
        </#if>
        },
    </#if>
    <#if (config.evalTrain)??>
        eval_train=${config.evalTrain?string("True","False")},
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
        </#list>}
    </#if>
    )
</#list>