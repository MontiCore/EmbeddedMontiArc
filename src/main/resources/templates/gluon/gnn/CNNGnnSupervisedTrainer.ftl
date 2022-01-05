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
import h5py
import importlib
<#if tc.architecture.useDgl>
import dgl
from dgl.data.utils import load_graphs
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

class SoftmaxCrossEntropyLossNoBatches(gluon.loss.Loss):
    def __init__(self, axis=-1, sparse_label=True, from_logits=False, weight=None, **kwargs):
        super(SoftmaxCrossEntropyLossNoBatches, self).__init__(weight, **kwargs)
        self._axis = axis
        self._sparse_label = sparse_label
        self._from_logits = from_logits

    def hybrid_forward(self, F, pred, label, sample_weight=None):
        if mx.is_np_array():
            log_softmax = F.npx.log_softmax
            pick = F.npx.pick
        else:
            log_softmax = F.log_softmax
            pick = F.pick
        if not self._from_logits:
            pred = log_softmax(pred, self._axis)
        if self._sparse_label:
            loss = -pick(pred, label, axis=self._axis, keepdims=True)
        else:
            label = mx._reshape_like(F, label, pred)
            loss = -(pred * label).sum(axis=self._axis, keepdims=True)
            loss = mx._apply_weighting(F, loss, self._weight, sample_weight)
        if mx.is_np_array():
            if F is mx.ndarray:
                return loss.mean(axis=tuple(range(1, loss.ndim)))
            else:
                return F.npx.batch_flatten(loss).mean(axis=1)
        else:
            return loss.mean()

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
        loss = F.slice(loss, begin=(self._mask[0]), end=(self._mask[1]))
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
            label = mx.nd.slice(label, begin=(self.mask[0]), end=(self.mask[1]))
            pred_label = mx.nd.slice(pred_label, begin=(self.mask[0]), end=(self.mask[1]))
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

# TODO update classname
class CNNGnnSupervisedTrainer:
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
        # single_pu_batch_size = int(batch_size/num_pus)

        if preprocessing:
            preproc_lib = "CNNPreprocessor_${tc.fileNameWithoutEnding?keep_after("CNNSupervisedTrainer_")}_executor"
            train_iter, test_iter, data_mean, data_std, train_images, test_images = self._data_loader.load_preprocessed_data(batch_size, preproc_lib, shuffle_data)
        else:
            data, graph, data_mean, data_std = self._data_loader.load_data()

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
            self._net_creator.construct(context=mx_context, data_mean=data_mean, data_std=data_std, graph=graph)
        else:
            self._net_creator.construct(context=mx_context, graph=graph)

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

        sparseLabel = loss_params['sparse_label'] if 'sparse_label' in loss_params else True
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
        print("If nothing gets printed after this remove tc.containsAdaNet()")
<#if tc.containsAdaNet()>
        print("Do not remove tc.containsAdaNet()")
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
        tic = None
        avg_speed = 0
        n = 0
    
        for epoch in range(begin_epoch, begin_epoch + num_epoch):
            if shuffle_data:
                # Preprocessing currently not supported
                if preprocessing:
                    preproc_lib = "CNNPreprocessor_${tc.fileNameWithoutEnding?keep_after("CNNSupervisedTrainer_")}_executor"
                    train_iter, test_iter, data_mean, data_std, train_images, test_images = self._data_loader.load_preprocessed_data(batch_size, preproc_lib, shuffle_data)
                else:
                    data, graph, label, data_mean, data_std = self._data_loader.load_data()

            global_loss_train = 0.0
            data.reset()

            loss_total = 0
<#if !(tc.architecture.useDgl)>
            loss_function = SoftmaxCrossEntropyLossMasked(axis=loss_axis, mask=train_mask, from_logits=fromLogits, sparse_label=sparseLabel, batch_axis=batch_axis)
            loss_function.hybridize()
</#if>
            for batch_i, batch in enumerate(data):
                data_index = 0
                with autograd.record():
<#include "gnnExecute.ftl">



                    losses = [0]*num_pus
                    for i in range(num_pus):
                        for element in lossList[i]:
                            losses[i] = losses[i] + element

                for loss in losses:
                    loss.backward()
                    loss_total += loss.sum().asscalar()
                    global_loss_train += loss.sum().asscalar()

                if clip_global_grad_norm:
                    grads = []

                    for network in self._networks.values():
                        grads.extend([param.grad(mx_context) for param in network.collect_params().values()])

                    gluon.utils.clip_global_norm(grads, clip_global_grad_norm)

                for trainer in trainers:
                    trainer.step(batch_size)

                if tic is None:
                    tic = time.time()
                else:
                    try:
                        speed = log_period * batch_size / (time.time() - tic)
                    except ZeroDivisionError:
                        speed = float("inf")

                    loss_total = 0

                    logging.info("Epoch[%d] Speed: %.2f samples/sec Loss: %.5f" % (epoch, speed, loss_total))
<#if tc.architecture.useDgl>
                    # logging does not work when importing dgl right now
                    print("Epoch[%d] Speed: %.2f samples/sec Loss: %.5f" % (epoch, speed, loss_total))
</#if>
                    avg_speed += speed
                    n += 1

                    tic = time.time()

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
<#if !(tc.architecture.useDgl)>
            metric = mx.metric.create(eval_metric, mask=test_mask, **eval_metric_params)
<#else>
            metric = mx.metric.create(eval_metric, **eval_metric_params)
</#if>

            predictions = []
            for output_name in outputs:
                if mx.nd.shape_array(mx.nd.squeeze(output_name)).size > 1:
                    predictions.append(mx.nd.argmax(output_name, axis=argmax_axis))
                else:
                    predictions.append(output_name)
<#if tc.architecture.useDgl>
            metric.update(preds=predictions[0], labels=mx.nd.squeeze(labels[0][0]), mask=graph.ndata['test_mask'])
<#else>
            metric.update(preds=predictions[0], labels=mx.nd.squeeze(labels[0][0]))
</#if>
            test_metric_name = metric.get()[0]
            test_metric_score = metric.get()[1]
            # global_loss_train /= batches

            metric_file = open(self._net_creator._model_dir_ + 'metric.txt', 'w')
            metric_file.write(test_metric_name + " " + str(test_metric_score))
            metric_file.close()

            logging.info("Epoch[%d] Test metric: %f, Train loss: %f" % (epoch, test_metric_score, global_loss_train))
<#if tc.architecture.useDgl>
            print("Epoch[%d] Test metric: %f, Train loss: %f" % (epoch, test_metric_score, global_loss_train))
</#if>
            if (epoch+1) % checkpoint_period == 0:
                for i, network in self._networks.items():
                    network.save_parameters(self.parameter_path(i) + '-' + str(epoch).zfill(4) + '.params')

        for i, network in self._networks.items():
            network.save_parameters(self.parameter_path(i) + '-' + str((num_epoch-1) + begin_epoch).zfill(4) + '.params')
<#if !(tc.architecture.useDgl)>
            network.export(self.parameter_path(i) + '_newest', epoch=0)
</#if>
            loss_function.export(self.parameter_path(i) + '_newest_loss', epoch=0)

    def parameter_path(self, index):
        return self._net_creator._model_dir_ + self._net_creator._model_prefix_ + '_' + str(index)
