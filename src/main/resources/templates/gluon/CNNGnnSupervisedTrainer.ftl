<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import mxnet as mx
import logging
import numpy as np
import time
import os
import shutil
import pickle
import math
import sys
import inspect
<#if tc.architecture.useDgl>
import dgl
</#if>
from mxnet import gluon, autograd, nd
<#if tc.containsAdaNet()>
from typing import List
from mxnet.gluon.loss import Loss, SigmoidBCELoss
from mxnet.ndarray import add, concatenate
sys.path.insert(1, '${tc.architecture.getAdaNetUtils()}')
#${tc.architecture.getAdaNetUtils()}
from adanet import fit
</#if>
try:
    import AdamW
except:
    pass


class CrossEntropyLoss(gluon.loss.Loss):
    def __init__(self, axis=-1, sparse_label=True, weight=None, batch_axis=0, **kwargs):
        super(CrossEntropyLoss, self).__init__(weight, batch_axis, **kwargs)
        self._axis = axis
        self._sparse_label = sparse_label

    def hybrid_forward(self, F, pred, label, sample_weight=None):
        pred = F.log(pred)
        if self._sparse_label:
            loss = -F.pick(pred, label, axis=self._axis, keepdims=True)
        else:
            label = gluon.loss._reshape_like(F, label, pred)
            loss = -F.sum(pred * label, axis=self._axis, keepdims=True)
        loss = gluon.loss._apply_weighting(F, loss, self._weight, sample_weight)
        return F.mean(loss, axis=self._batch_axis, exclude=True)


class SoftmaxCrossEntropyLossMasked(gluon.loss.Loss):
    def __init__(self, axis=-1, mask=None, sparse_label=True, from_logits=False, weight=None, batch_axis=0, **kwargs):
        super(SoftmaxCrossEntropyLossMasked, self).__init__(weight, batch_axis, **kwargs)
        self._axis = axis
        self._sparse_label = sparse_label
        self._from_logits = from_logits
        self._mask = mask

    def hybrid_forward(self, F, pred, label, sample_weight=None):
        log_softmax = F.log_softmax
        pick = F.pick
        if not self._from_logits:
            pred = log_softmax(pred, self._axis)
        if self._sparse_label:
            loss = -pick(pred, label, axis=self._axis, keepdims=True)
        else:
            label = gluon.loss._reshape_like(F, label, pred)
            loss = -(pred * label).sum(axis=self._axis, keepdims=True)
        # Masking
        loss = F.slice(loss, begin=(None, self._mask[0]), end=(None, self._mask[1]))
        return loss.mean(axis=self._batch_axis, exclude=True)


@mx.metric.register
class ACCURACY_MASKED(mx.metric.EvalMetric):

    def __init__(self, axis=1, mask=None, name='accuracy_masked',
                 output_names=None, label_names=None):
        super(ACCURACY_MASKED, self).__init__(
            name, axis=axis,
            output_names=output_names, label_names=label_names)
        self.axis = axis
        self.mask = mask

    def update(self, labels, preds):
        mx.metric.check_label_shapes(labels, preds)

        for label, pred_label in zip(labels, preds):
            if pred_label.shape != label.shape:
                pred_label = mx.nd.argmax(pred_label, axis=self.axis, keepdims=True)
            label = label.astype('int32')
            pred_label = pred_label.astype('int32').as_in_context(label.context)

            mx.metric.check_label_shapes(label, pred_label)
            label = mx.nd.slice(label, begin=(None, self.mask[0]), end=(None, self.mask[1]))
            pred_label = mx.nd.slice(pred_label, begin=(None, self.mask[0]), end=(None, self.mask[1]))
            correct = mx.nd.sum(mx.nd.equal(label, pred_label)).asscalar()
            total = mx.nd.sum(mx.nd.ones_like(label)).asscalar()

            self.sum_metric += correct
            self.num_inst += total

@mx.metric.register
class ACCURACY_DGL(mx.metric.EvalMetric):

    def __init__(self, axis=1, name='accuracy_dgl',
                 output_names=None, label_names=None):
        super(ACCURACY_DGL, self).__init__(
            name, axis=axis,
            output_names=output_names, label_names=label_names)
        self.axis = axis

    def update(self, labels, preds, mask):
        mx.metric.check_label_shapes(labels, preds)

        if preds.shape != labels.shape:
            preds = mx.nd.argmax(preds, axis=self.axis, keepdims=True)
        labels = labels.astype('int32')
        preds = preds.astype('int32').as_in_context(labels.context)
        mask = mask.astype('int32').as_in_context(labels.context)

        mx.metric.check_label_shapes(labels, preds)

        correct = ((preds == labels)*mask).sum().asscalar()
        total = mask.sum().asscalar()

        self.sum_metric += correct
        self.num_inst += total

class ${tc.fileNameWithoutEnding}:
    def __init__(self, data_loader, net_constructor):
        self._data_loader = data_loader
        self._net_creator = net_constructor
        <#if tc.containsAdaNet()>
        self._dataClass = net_constructor.dataClass
        self.AdaNet = ${tc.containsAdaNet()?string('True','False')}
        </#if>
        self._networks = {}

    def train(self, batch_size=64,
              num_epoch=10,
              eval_metric='acc',
              eval_metric_params={},
              eval_train=False,
              loss ='softmax_cross_entropy',
              loss_params={},
              optimizer='adam',
              optimizer_params=(('learning_rate', 0.001),),
              load_checkpoint=True,
              checkpoint_period=5,
              load_pretrained=False,
              log_period=50,
              context='gpu',
              save_attention_image=False,
              use_teacher_forcing=False,
              normalize=True,
              shuffle_data=False,
              clip_global_grad_norm=None,
              preprocessing=False,
              use_dgl=False,
              argmax_axis=1,
              train_mask=None,
              test_mask=None):
        num_pus = 1
        if context == 'gpu':
            num_pus = mx.context.num_gpus()
            if num_pus >= 1:
                if num_pus == 1:
                    mx_context = [mx.gpu(0)]
                else:
                    mx_context = [mx.gpu(i) for i in range(num_pus)]
            else:
                logging.error("Context argument is '" + context + "'. But no gpu is present in the system.")
        elif context == 'cpu':
            mx_context = [mx.cpu()]
        else:
            logging.error("Context argument is '" + context + "'. Only 'cpu' and 'gpu are valid arguments'.")
        single_pu_batch_size = int(batch_size/num_pus)

        if preprocessing:
            preproc_lib = "CNNPreprocessor_${tc.fileNameWithoutEnding?keep_after("CNNSupervisedTrainer_")}_executor"
            train_iter, test_iter, data_mean, data_std, train_images, test_images = self._data_loader.load_preprocessed_data(batch_size, preproc_lib, shuffle_data)
        else:
            train_iter, test_iter, graph_, data_mean, data_std, train_images, test_images = self._data_loader.load_data(batch_size, shuffle_data)

        if 'weight_decay' in optimizer_params:
            optimizer_params['wd'] = optimizer_params['weight_decay']
            del optimizer_params['weight_decay']
        if 'learning_rate_decay' in optimizer_params:
            min_learning_rate = 1e-08
            if 'learning_rate_minimum' in optimizer_params:
                min_learning_rate = optimizer_params['learning_rate_minimum']
                del optimizer_params['learning_rate_minimum']
            optimizer_params['lr_scheduler'] = mx.lr_scheduler.FactorScheduler(
                                                   optimizer_params['step_size'],
                                                   factor=optimizer_params['learning_rate_decay'],
                                                   stop_factor_lr=min_learning_rate)
            del optimizer_params['step_size']
            del optimizer_params['learning_rate_decay']

        if normalize:
            self._net_creator.construct(context=mx_context, data_mean=data_mean, data_std=data_std)
        else:
            self._net_creator.construct(context=mx_context)

        begin_epoch = 0
        if load_checkpoint:
            begin_epoch = self._net_creator.load(mx_context)
        elif load_pretrained:
            self._net_creator.load_pretrained_weights(mx_context)
        else:
            if os.path.isdir(self._net_creator._model_dir_):
                shutil.rmtree(self._net_creator._model_dir_)

        self._networks = self._net_creator.networks

        try:
            os.makedirs(self._net_creator._model_dir_)
        except OSError:
            if not os.path.isdir(self._net_creator._model_dir_):
                raise

        if optimizer == "adamw":
            trainers = [mx.gluon.Trainer(network.collect_params(), AdamW.AdamW(**optimizer_params)) for network in self._networks.values() if len(network.collect_params().values()) != 0]
        else:
            trainers = [mx.gluon.Trainer(network.collect_params(), optimizer, optimizer_params) for network in self._networks.values() if len(network.collect_params().values()) != 0]

        margin = loss_params['margin'] if 'margin' in loss_params else 1.0
        sparseLabel = loss_params['sparse_label'] if 'sparse_label' in loss_params else True
        ignore_indices = [loss_params['ignore_indices']] if 'ignore_indices' in loss_params else []
        loss_axis = loss_params['loss_axis'] if 'loss_axis' in loss_params else -1
        batch_axis = loss_params['batch_axis'] if 'batch_axis' in loss_params else 0
        if loss == 'softmax_cross_entropy':
            fromLogits = loss_params['from_logits'] if 'from_logits' in loss_params else False
            loss_function = mx.gluon.loss.SoftmaxCrossEntropyLoss(axis=loss_axis, from_logits=fromLogits, sparse_label=sparseLabel, batch_axis=batch_axis)
        elif loss == 'softmax_cross_entropy_masked':
            fromLogits = loss_params['from_logits'] if 'from_logits' in loss_params else False
            loss_function = SoftmaxCrossEntropyLossMasked(axis=loss_axis, mask=train_mask, from_logits=fromLogits, sparse_label=sparseLabel, batch_axis=batch_axis)
        elif loss == 'cross_entropy':
            loss_function = CrossEntropyLoss(axis=loss_axis, sparse_label=sparseLabel, batch_axis=batch_axis)
        else:
            logging.error("Invalid loss parameter.")

        loss_function.hybridize()
<#if tc.containsAdaNet()>
<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.containsAdaNet()>
        assert self._networks[${networkInstruction?index}].AdaNet, "passed model is not an AdaNet model"
        self._networks[${networkInstruction?index}] = fit(
                    loss=loss_function,
                    optimizer=optimizer,
                    epochs=num_epoch,
                    optimizer_params = optimizer_params,
                    train_iter = train_iter,
                    data_class = self._dataClass[${networkInstruction?index}],
                    batch_size=batch_size,
                    ctx=mx_context[0],
                    logging=logging
                )
        logging.info(self._networks[${networkInstruction?index}])
        logging.info(f"node count: {self._networks[${networkInstruction?index}].get_node_count()}")
</#if>
        # update trainers
        if optimizer == "adamw":
            trainers = [mx.gluon.Trainer(network.collect_params(), AdamW.AdamW(**optimizer_params)) for network in self._networks.values() if len(network.collect_params().values()) != 0]
        else:
            trainers = [mx.gluon.Trainer(network.collect_params(), optimizer, optimizer_params) for network in self._networks.values() if len(network.collect_params().values()) != 0]
</#list>
</#if>
<#list tc.architecture.networkInstructions as networkInstruction>    
<#if networkInstruction.body.episodicSubNetworks?has_content>
<#assign episodicReplayVisited = true>
</#if>
</#list>    

<#if episodicReplayVisited??>
        #Episodic memory Replay
        episodic_layers = {}
        episodic_store_buffer = {}
        episodic_query_networks = {}
        store_prob = {}
<#list tc.architecture.networkInstructions as networkInstruction>
        episodic_layers[${networkInstruction?index}] = []
        episodic_store_buffer[${networkInstruction?index}] = []
        episodic_query_networks[${networkInstruction?index}] = []
        store_prob[${networkInstruction?index}] = []
<#list networkInstruction.body.episodicSubNetworks as elements>
<#if elements?index != 0>
        sub_net = self._networks[${networkInstruction?index}].episodic_sub_nets[${elements?index - 1}]
        layer = [param for param in inspect.getmembers(sub_net, lambda x: not(inspect.isroutine(x))) if param[0].startswith("memory")][0][1]
        episodic_layers[0].append(layer)
        episodic_store_buffer[${networkInstruction?index}].append([])
        episodic_query_networks[0].append(episodic_layers[0][-1].get_query_network(mx_context))
        store_prob[${networkInstruction?index}].append(nd.array([1-episodic_layers[0][-1].store_prob, episodic_layers[0][-1].store_prob], ctx=mx.cpu()))                                                                                  
</#if>
</#list>
</#list>
        # Episodic memory local adaptation
        local_adaptation_loss_function = LocalAdaptationLoss(lamb=0.001)
</#if>

        tic = None

        avg_speed = 0
        n = 0
    
        for epoch in range(begin_epoch, begin_epoch + num_epoch):
            if shuffle_data:
                if preprocessing:
                    preproc_lib = "CNNPreprocessor_${tc.fileNameWithoutEnding?keep_after("CNNSupervisedTrainer_")}_executor"
                    train_iter, test_iter, data_mean, data_std, train_images, test_images = self._data_loader.load_preprocessed_data(batch_size, preproc_lib, shuffle_data)
                else:
                    train_iter, test_iter, graph_, data_mean, data_std, train_images, test_images = self._data_loader.load_data(batch_size, shuffle_data)

            global_loss_train = 0.0
            train_batches = 0

            loss_total = 0
            train_iter.reset()
            if train_mask != None:
                loss_function = SoftmaxCrossEntropyLossMasked(axis=loss_axis, mask=train_mask, from_logits=fromLogits, sparse_label=sparseLabel, batch_axis=batch_axis)
                loss_function.hybridize()
            for batch_i, batch in enumerate(train_iter):
                
<#include "pythonEpisodicExecuteTrain.ftl">
                                 
                with autograd.record():
<#include "pythonExecuteTrain.ftl">

                    losses = [0]*num_pus
                    for i in range(num_pus):
                        for element in lossList[i]:
                            losses[i] = losses[i] + element

                for loss in losses: 
                    loss.backward()
                    loss_total += loss.sum().asscalar()
                    global_loss_train += loss.sum().asscalar()

                train_batches += 1

                if clip_global_grad_norm:
                    grads = []

                    for network in self._networks.values():
                        grads.extend([param.grad(mx_context) for param in network.collect_params().values()])

                    gluon.utils.clip_global_norm(grads, clip_global_grad_norm)

                for trainer in trainers:
                    trainer.step(batch_size)
    
<#if episodicReplayVisited??>                                                  
                #storing samples for episodic replay
                for net_i in range(len(self._networks)):
                    for layer_i, layer in enumerate(episodic_layers[net_i]):
                        layer.store_samples(episodic_store_buffer[net_i][layer_i], labels, episodic_query_networks[net_i][layer_i], store_prob[net_i][layer_i], mx_context)

</#if>
                if tic is None:
                    tic = time.time()
                else:
                    if batch_i % log_period == 0:
                        try:
                            speed = log_period * batch_size / (time.time() - tic)
                        except ZeroDivisionError:
                            speed = float("inf")

                        loss_avg = loss_total / (batch_size * log_period)
                        loss_total = 0

                        logging.info("Epoch[%d] Batch[%d] Speed: %.2f samples/sec Loss: %.5f" % (epoch, batch_i, speed, loss_avg))
                        # logging does not work when importing dgl right now
                        if use_dgl:
                            print("Epoch[%d] Batch[%d] Speed: %.2f samples/sec Loss: %.5f" % (epoch, batch_i, speed, loss_avg))

                        avg_speed += speed
                        n += 1
    
                        tic = time.time()

            global_loss_train /= (train_batches * batch_size)

            tic = None
<#assign containsUnrollNetwork = false>
<#assign anyEpisodicLocalAdaptation = false>
<#list tc.architecture.networkInstructions as networkInstruction>    
<#if networkInstruction.isUnroll()>
<#assign containsUnrollNetwork = true>
</#if>
<#if networkInstruction.body.anyEpisodicLocalAdaptation>
<#assign anyEpisodicLocalAdaptation = true>
</#if>
</#list>
    
<#if episodicReplayVisited?? && anyEpisodicLocalAdaptation>                          
            params = {}
            for key in self._networks:
                paramDict = self._networks[key].collect_params()
                params[key] = {}
                for param in paramDict:
                    params[key][param] = paramDict[param].data(ctx=mx_context[0]).copy()
</#if>
    
            if eval_train:
                train_iter.batch_size = single_pu_batch_size
                train_iter.reset()
                if train_mask != None:
                    metric = mx.metric.create(eval_metric, mask=train_mask, **eval_metric_params)
                else:
                    metric = mx.metric.create(eval_metric, **eval_metric_params)
                for batch_i, batch in enumerate(train_iter):

<#if episodicReplayVisited?? && anyEpisodicLocalAdaptation && !containsUnrollNetwork>
<#include "pythonExecuteTest.ftl">

                        predictions = []
                        for output_name in outputs:
                            if mx.nd.shape_array(mx.nd.squeeze(output_name)).size > 1:
                                predictions.append(mx.nd.argmax(output_name, axis=argmax_axis))
                            else:
                                predictions.append(output_name)

                        if use_dgl:
                            metric.update(preds=predictions[0], labels=mx.nd.squeeze(labels[0]), mask=graph_.ndata['train_mask'])
                        else:
                            metric.update(preds=predictions, labels=[labels[j][local_adaptation_batch_i] for j in range(len(labels))])
<#list tc.architecture.networkInstructions as networkInstruction>    
                self._networks[${networkInstruction?index}].collect_params().load_dict(params[${networkInstruction?index}], ctx=mx_context[0])
</#list>      
<#else>
<#include "pythonExecuteTest.ftl">
    
<#include "saveAttentionImageTrain.ftl">

                    predictions = []
                    for output_name in outputs:
                        if mx.nd.shape_array(mx.nd.squeeze(output_name)).size > 1:
                            predictions.append(mx.nd.argmax(output_name, axis=argmax_axis))
                        else:
                            predictions.append(output_name)
                    if use_dgl:
                        metric.update(preds=predictions[0], labels=mx.nd.squeeze(labels[0]), mask=graph_.ndata['train_mask'])
                    else:
                        metric.update(preds=predictions, labels=[labels[j] for j in range(len(labels))])
</#if>

                train_metric_score = metric.get()[1]
            else:
                train_metric_score = 0

            global_loss_test = 0.0
            test_batches = 0
    
            test_iter.batch_size = single_pu_batch_size
            test_iter.reset()
            if test_mask != None:
                loss_function = SoftmaxCrossEntropyLossMasked(axis=loss_axis, mask=test_mask, from_logits=fromLogits, sparse_label=sparseLabel, batch_axis=batch_axis)
                loss_function.hybridize()
                metric = mx.metric.create(eval_metric, mask=test_mask, **eval_metric_params)
            else:
                metric = mx.metric.create(eval_metric, **eval_metric_params)

            for batch_i, batch in enumerate(test_iter):
                if True: <#-- Fix indentation -->
                                                   
<#if episodicReplayVisited?? && anyEpisodicLocalAdaptation && !containsUnrollNetwork>
<#include "pythonExecuteTest.ftl">
    
                        loss = 0
                        for element in lossList:
                            loss = loss + element
                        global_loss_test += loss.sum().asscalar()

                        test_batches += 1
                
                        predictions = []
                        for output_name in outputs:
                            if mx.nd.shape_array(mx.nd.squeeze(output_name)).size > 1:
                                predictions.append(mx.nd.argmax(output_name, axis=argmax_axis))
                            else:
                                predictions.append(output_name)
                        if use_dgl:
                            metric.update(preds=predictions[0], labels=mx.nd.squeeze(labels[0]), mask=graph_.ndata['test_mask'])
                        else:
                            metric.update(preds=predictions, labels=[labels[j][local_adaptation_batch_i] for j in range(len(labels))])
<#list tc.architecture.networkInstructions as networkInstruction>    
                self._networks[${networkInstruction?index}].collect_params().load_dict(params[${networkInstruction?index}], ctx=mx_context[0])
</#list>
            global_loss_test /= (test_batches)    
<#else>
<#include "pythonExecuteTest.ftl">

<#include "saveAttentionImageTest.ftl">

                loss = 0
                for element in lossList:
                    loss = loss + element

                global_loss_test += loss.sum().asscalar()

                test_batches += 1

                predictions = []
                for output_name in outputs:
                    if mx.nd.shape_array(mx.nd.squeeze(output_name)).size > 1:
                        predictions.append(mx.nd.argmax(output_name, axis=argmax_axis))
                    else:
                        predictions.append(output_name)
                if use_dgl:
                    metric.update(preds=predictions[0], labels=mx.nd.squeeze(labels[0]), mask=graph_.ndata['test_mask'])
                else:
                    metric.update(preds=predictions, labels=[labels[j] for j in range(len(labels))])

            global_loss_test /= (test_batches * single_pu_batch_size)
</#if>
            test_metric_name = metric.get()[0]
            test_metric_score = metric.get()[1]

            metric_file = open(self._net_creator._model_dir_ + 'metric.txt', 'w')
            metric_file.write(test_metric_name + " " + str(test_metric_score))
            metric_file.close()

            logging.info("Epoch[%d] Train metric: %f, Test metric: %f, Train loss: %f, Test loss: %f" % (epoch, train_metric_score, test_metric_score, global_loss_train, global_loss_test))
            # when importing dgl logging does not work right now
            if use_dgl:
                print("Epoch[%d] Train metric: %f, Test metric: %f, Train loss: %f, Test loss: %f" % (epoch, train_metric_score, test_metric_score, global_loss_train, global_loss_test))

            if (epoch+1) % checkpoint_period == 0:
                for i, network in self._networks.items():
                    network.save_parameters(self.parameter_path(i) + '-' + str(epoch).zfill(4) + '.params')
                    if hasattr(network, 'episodic_sub_nets'):
                        for j, net in enumerate(network.episodic_sub_nets):
                            episodic_layers[i][j].save_memory(self.parameter_path(i) + "_episodic_memory_sub_net_" + str(j + 1) + "-" + str(epoch).zfill(4))

        for i, network in self._networks.items():
            network.save_parameters(self.parameter_path(i) + '-' + str((num_epoch-1) + begin_epoch).zfill(4) + '.params')
            if not use_dgl:
                network.export(self.parameter_path(i) + '_newest', epoch=0)
            
            if hasattr(network, 'episodic_sub_nets'):
                network.episodicsubnet0_.export(self.parameter_path(i) + '_newest_episodic_sub_net_' + str(0), epoch=0)
                for j, net in enumerate(network.episodic_sub_nets):
                    net.export(self.parameter_path(i) + '_newest_episodic_sub_net_' + str(j+1), epoch=0)
                    episodic_query_networks[i][j].export(self.parameter_path(i) + '_newest_episodic_query_net_' + str(j+1), epoch=0)
                    episodic_layers[i][j].save_memory(self.parameter_path(i) + "_episodic_memory_sub_net_" + str(j + 1) + "-" + str((num_epoch - 1) + begin_epoch).zfill(4))
                    episodic_layers[i][j].save_memory(self.parameter_path(i) + "_newest_episodic_memory_sub_net_" + str(j + 1) + "-0000")
            loss_function.export(self.parameter_path(i) + '_newest_loss', epoch=0)

    def parameter_path(self, index):
        return self._net_creator._model_dir_ + self._net_creator._model_prefix_ + '_' + str(index)
