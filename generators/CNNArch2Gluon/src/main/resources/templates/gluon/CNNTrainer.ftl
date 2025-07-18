<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import logging
import pathlib

import mxnet as mx

<#list configurations as config>
import CNNCreator_${config.instanceName}
import CNNDataLoader_${config.instanceName}
from CNNDatasets_${config.instanceName} import RetrainingConf
import CNNDataCleaner_${config.instanceName}
import CNNSupervisedTrainer_${config.instanceName}
<#if (config.optimizer)?? && (config.optimizerName == "hpo")>
import CNNHyperparameterOptimization_${config.instanceName}
</#if>
</#list>

if __name__ == "__main__":
    logger = logging.getLogger()
    logging.basicConfig(level=logging.DEBUG)
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

<#list configurations as config>
    ${config.instanceName}_creator = CNNCreator_${config.instanceName}.CNNCreator_${config.instanceName}()
    ${config.instanceName}_creator.validate_parameters()
    ${config.instanceName}_cleaner = CNNDataCleaner_${config.instanceName}.CNNDataCleaner_${config.instanceName}()
    ${config.instanceName}_loader = CNNDataLoader_${config.instanceName}.CNNDataLoader_${config.instanceName}(
        ${config.instanceName}_cleaner
    )

    prev_dataset = None
    retraining_conf = ${config.instanceName}_loader.load_retraining_conf()
    
<#--  Hyperparameter Optimization Code  -->
<#if (config.optimizer)?? && (config.optimizerName == "hpo")>
    dataset = retraining_conf.changes[0]
    ${config.instanceName}_creator.dataset = dataset

    ${config.instanceName}_hyperparameter_tuner = CNNHyperparameterOptimization_${config.instanceName}.CNNHyperparameterOptimization_${config.instanceName}(
        ${config.instanceName}_loader,
        ${config.instanceName}_creator
    )
    
    ${config.instanceName}_hyperparameter_tuner.hyperparameter_optimization(
    <#if (config.batchSize)??>
        batch_size=${config.batchSize},
    </#if>
    <#if (config.numEpoch)??>
        num_epoch=${config.numEpoch},
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
            <#if param == "rotation_angle">
            '${param}': [<#list config.dataImbalanceParameters[param] as lr>${lr}<#if lr?has_next>, </#if></#list>],
            <#else>
            '${param}': ${config.dataImbalanceParameters[param]}<#sep>,
            </#if>
        </#list>
        </#if>
        },
    </#if>
    <#if (config.dataSplitting)??>
        data_splitting='${config.dataSplittingName}',
        data_splitting_params={
        <#if (config.dataSplittingParameters)??>
        <#list config.dataSplittingParameters?keys as param>
            <#if param == "tvt_ratio">
            '${param}': [<#list config.dataSplittingParameters[param] as r>${r}<#if r?has_next>, </#if></#list>],
            <#else>
            '${param}': ${config.dataSplittingParameters[param]}<#sep>,
            </#if>
        </#list>
        </#if>
        },
    </#if>
    <#if (config.optimizer)??>
        optimizer='${config.optimizerName}',
        optimizer_params={
        <#if (config.optimizerParameters)??>
        <#if (config.optimizerParameters['learning_rate_range'])??>
            'learning_rate_range': [<#list config.optimizerParameters['learning_rate_range'] as lr>${lr}<#if lr?has_next>, </#if></#list>],
        </#if>
        <#if (config.optimizerParameters['weight_decay_range'])??>
            'weight_decay_range': [<#list config.optimizerParameters['weight_decay_range'] as wd>${wd}<#if wd?has_next>, </#if></#list>],
        </#if>
        <#if (config.optimizerParameters['momentum_range'])??>
            'momentum_range': [<#list config.optimizerParameters['momentum_range'] as m>${m}<#if m?has_next>, </#if></#list>],
        </#if>
        <#if (config.optimizerParameters['optimizer_options'])??>
            'optimizer_options': [<#list config.optimizerParameters['optimizer_options'] as t>"${t}"<#if t?has_next>, </#if></#list>],
        </#if>
        <#if (config.optimizerParameters['with_cleaning'])??>
            'with_cleaning': ${config.optimizerParameters['with_cleaning']},
        </#if>
        <#if (config.optimizerParameters['ntrials'])??>
            'ntrials': ${config.optimizerParameters['ntrials']}<#sep>,
        </#if>
        </#if>
        },
    </#if>
        dataset=dataset,
        test_dataset=retraining_conf.testing,
        val_dataset=retraining_conf.validating
    )

<#--  Supervised Trainer Code  -->
<#else>
    ${config.instanceName}_trainer = CNNSupervisedTrainer_${config.instanceName}.CNNSupervisedTrainer_${config.instanceName}(
        ${config.instanceName}_loader,
        ${config.instanceName}_creator
    )

    for dataset in retraining_conf.changes:
        ${config.instanceName}_creator.dataset = dataset
        if(dataset.retraining):
            if prev_dataset: 
                logger.info("Retrain dataset %s on top of dataset %s.", dataset.id, prev_dataset.id)
            else: 
                logger.info("Dataset %s needs to be trained. Hash was different during the last EMADL2CPP run.", dataset.id)

            <#if (config.optimizer)??>
            optimizer = '${config.optimizerName}'
            optimizer_params = {
            <#list config.optimizerParameters?keys as param>
                <#assign paramName = param>
                <#assign paramValue = config.optimizerParameters[param]>
                <#if param == "learning_rate_policy">
                    <#assign paramValue = "'${paramValue}'">
                </#if>
                '${paramName}': ${paramValue}<#sep>,
            </#list>}
            </#if>

            <#if (config.retrainingOptimizer)??>
            <#if (config.retrainingType)?? && config.retrainingType="manually">
            if prev_dataset:
                optimizer = '${config.retrainingOptimizerName}'
                optimizer_params = {
                <#list config.retrainingOptimizerParameters?keys as param>
                    <#assign paramName = param>
                    <#assign paramValue = config.optimizerParameters[param]>
                    <#if param == "learning_rate_policy">
                        <#assign paramValue = "'${paramValue}'">
                    </#if>
                    '${paramName}': ${paramValue}<#sep>,
                </#list>}
            </#if>
            </#if>

            <#if (config.retraining)??>
            <#if (config.retrainingName != "ignore")??>
            if prev_dataset:
                optimizer = ''
                optimizer_params = {
                <#list config.retrainingOptimizerParameters?keys as param>
                    <#assign paramName = param>
                    <#assign paramValue = config.optimizerParameters[param]>
                    <#if param == "learning_rate_policy">
                        <#assign paramValue = "'${paramValue}'">
                    </#if>
                    '${paramName}': ${paramValue}<#sep>,
                </#list>}
            </#if>
            </#if>

            ${config.instanceName}_trainer.train(
                dataset=dataset,
                test_dataset=retraining_conf.testing,
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
                load_pretrained_dataset=dataset,
            <#else>
                load_pretrained=bool(prev_dataset),
                load_pretrained_dataset=prev_dataset,
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
                    <#if param == "rotation_angle">
                    '${param}': [<#list config.dataImbalanceParameters[param] as lr>${lr}<#if lr?has_next>, </#if></#list>],
                    <#else>
                    '${param}': ${config.dataImbalanceParameters[param]}<#sep>,
                    </#if>
                </#list>
                </#if>
                },
            </#if>
            <#if (config.dataSplitting)??>
                data_splitting='${config.dataSplittingName}',
                data_splitting_params={
                <#if (config.dataSplittingParameters)??>
                <#list config.dataSplittingParameters?keys as param>
                    <#if param == "tvt_ratio">
                    '${param}': [<#list config.dataSplittingParameters[param] as r>${r}<#if r?has_next>, </#if></#list>],
                    <#else>
                    '${param}': ${config.dataSplittingParameters[param]}<#sep>,
                    </#if>
                </#list>
                </#if>
                },
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
            <#if (config.multiGraph)??>
                multi_graph='${config.multiGraph?string("True","False")}',
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
            <#if (config.onnxExport)??>
                onnx_export=${config.onnxExport?string("True","False")},
            </#if>
            optimizer=optimizer,
            optimizer_params=optimizer_params,
            <#if (config.retrainingType)??>
            retraining_type='${config.retrainingType}',
            </#if>
            )
        else: 
            logger.info("Skipped training of dataset %s. Training is not necessary", dataset.id)

        prev_dataset = dataset
</#if>

</#list>
