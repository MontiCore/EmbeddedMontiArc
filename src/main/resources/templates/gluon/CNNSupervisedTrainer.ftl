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
from mxnet import gluon, autograd, nd
<#if tc.containsAdaNet()>
from mxnet.gluon.loss import Loss, SigmoidBCELoss
from mxnet.ndarray import add, concatenate
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

class LogCoshLoss(gluon.loss.Loss):
    def __init__(self, weight=None, batch_axis=0, **kwargs):
        super(LogCoshLoss, self).__init__(weight, batch_axis, **kwargs)

    def hybrid_forward(self, F, pred, label, sample_weight=None):
        loss = F.log(F.cosh(pred - label))
        loss = gluon.loss._apply_weighting(F, loss, self._weight, sample_weight)
        return F.mean(loss, axis=self._batch_axis, exclude=True)

class SoftmaxCrossEntropyLossIgnoreIndices(gluon.loss.Loss):
    def __init__(self, axis=-1, ignore_indices=[], sparse_label=True, from_logits=False, weight=None, batch_axis=0, **kwargs):
        super(SoftmaxCrossEntropyLossIgnoreIndices, self).__init__(weight, batch_axis, **kwargs)
        self._axis = axis
        self._ignore_indices = ignore_indices
        self._sparse_label = sparse_label
        self._from_logits = from_logits

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
        # ignore some indices for loss, e.g. <pad> tokens in NLP applications
        for i in self._ignore_indices:
            loss = F.broadcast_mul(loss, F.logical_not(F.broadcast_equal(F.argmax(pred, axis=1), F.ones_like(F.argmax(pred, axis=1))*i) * F.broadcast_equal(F.argmax(pred, axis=1), label)))
        return loss.mean(axis=self._batch_axis, exclude=True)

class DiceLoss(gluon.loss.Loss):
    def __init__(self, axis=-1, sparse_label=True, from_logits=False, weight=None,
                 batch_axis=0, **kwargs):
        super(DiceLoss, self).__init__(weight, batch_axis, **kwargs)
        self._axis = axis
        self._sparse_label = sparse_label
        self._from_logits = from_logits

    def dice_loss(self, F, pred, label):
        smooth = 1.
        pred_y = F.argmax(pred, axis = self._axis)
        intersection = pred_y * label
        score = (2 * F.mean(intersection, axis=self._batch_axis, exclude=True) + smooth) \
            / (F.mean(label, axis=self._batch_axis, exclude=True) + F.mean(pred_y, axis=self._batch_axis, exclude=True) + smooth)

        return - F.log(score)

    def hybrid_forward(self, F, pred, label, sample_weight=None):
        if not self._from_logits:
            pred = F.log_softmax(pred, self._axis)
        if self._sparse_label:
            loss = -F.pick(pred, label, axis=self._axis, keepdims=True)
        else:
            label = gluon.loss._reshape_like(F, label, pred)
            loss = -F.sum(pred*label, axis=self._axis, keepdims=True)
        loss = gluon.loss._apply_weighting(F, loss, self._weight, sample_weight)
        diceloss = self.dice_loss(F, pred, label)
        return F.mean(loss, axis=self._batch_axis, exclude=True) + diceloss

class SoftmaxCrossEntropyLossIgnoreLabel(gluon.loss.Loss):
    def __init__(self, axis=-1, from_logits=False, weight=None,
                 batch_axis=0, ignore_label=255, **kwargs):
        super(SoftmaxCrossEntropyLossIgnoreLabel, self).__init__(weight, batch_axis, **kwargs)
        self._axis = axis
        self._from_logits = from_logits
        self._ignore_label = ignore_label

    def hybrid_forward(self, F, output, label, sample_weight=None):
        if not self._from_logits:
            output = F.log_softmax(output, axis=self._axis)

        valid_label_map = (label != self._ignore_label)
        loss = -(F.pick(output, label, axis=self._axis, keepdims=True) * valid_label_map )

        loss = gluon.loss._apply_weighting(F, loss, self._weight, sample_weight)
        return F.sum(loss) / F.sum(valid_label_map)

class LocalAdaptationLoss(gluon.loss.Loss):
    def __init__(self, lamb, axis=-1, sparse_label=True, weight=None, batch_axis=0,  **kwargs):
        super(LocalAdaptationLoss, self).__init__(weight, batch_axis, **kwargs)
        self.lamb = lamb
        self._axis = axis
        self._sparse_label = sparse_label

    def hybrid_forward(self, F, pred, label, curr_weights, base_weights, sample_weight=None):
        pred = F.log(pred)
        if self._sparse_label:
            cross_entr_loss = -F.pick(pred, label, axis=self._axis, keepdims=True)
        else:
            label = gluon.loss._reshape_like(F, label, pred)
            cross_entr_loss = -F.sum(pred * label, axis=self._axis, keepdims=True)
        cross_entr_loss = F.mean(cross_entr_loss, axis=self._batch_axis, exclude=True)

        weight_diff_loss = 0
        for param_key in base_weights:
            weight_diff_loss = F.add(weight_diff_loss, F.norm(curr_weights[param_key] - base_weights[param_key]))

        #this check is neccessary, otherwise if weight_diff_loss is zero (first training iteration)
        #the trainer would update the networks weights to nan, this must have somthing to do how
        #mxnet internally calculates the derivatives / tracks the weights
        if weight_diff_loss > 0:
            loss = self.lamb * weight_diff_loss + cross_entr_loss
            loss = gluon.loss._apply_weighting(F, loss, self._weight, sample_weight)
        else:
            loss = gluon.loss._apply_weighting(F, cross_entr_loss, self._weight, sample_weight)

        return loss    

@mx.metric.register
class ACCURACY_IGNORE_LABEL(mx.metric.EvalMetric):
    """Ignores a label when computing accuracy.
    """
    def __init__(self, axis=1, metric_ignore_label=255, name='accuracy',
                 output_names=None, label_names=None):
        super(ACCURACY_IGNORE_LABEL, self).__init__(
            name, axis=axis,
            output_names=output_names, label_names=label_names)
        self.axis = axis
        self.ignore_label = metric_ignore_label

    def update(self, labels, preds):
        mx.metric.check_label_shapes(labels, preds)

        for label, pred_label in zip(labels, preds):
            if pred_label.shape != label.shape:
                pred_label = mx.nd.argmax(pred_label, axis=self.axis, keepdims=True)
            label = label.astype('int32')
            pred_label = pred_label.astype('int32').as_in_context(label.context)

            mx.metric.check_label_shapes(label, pred_label)

            correct = mx.nd.sum( (label == pred_label) * (label != self.ignore_label) ).asscalar()
            total = mx.nd.sum( (label != self.ignore_label) ).asscalar()

            self.sum_metric += correct
            self.num_inst += total

@mx.metric.register
class BLEU(mx.metric.EvalMetric):
    N = 4

    def __init__(self, exclude=None, name='bleu', output_names=None, label_names=None):
        super(BLEU, self).__init__(name=name, output_names=output_names, label_names=label_names)

        self._exclude = exclude or []

        self._match_counts = [0 for _ in range(self.N)]
        self._counts = [0 for _ in range(self.N)]

        self._size_ref = 0
        self._size_hyp = 0

    def update(self, labels, preds):
        labels, preds = mx.metric.check_label_shapes(labels, preds, True)

        new_labels = self._convert(labels)
        new_preds = self._convert(preds)

        for label, pred in zip(new_labels, new_preds):
            reference = [word for word in label if word not in self._exclude]
            hypothesis = [word for word in pred if word not in self._exclude]

            self._size_ref += len(reference)
            self._size_hyp += len(hypothesis)

            for n in range(self.N):
                reference_ngrams = self._get_ngrams(reference, n + 1)
                hypothesis_ngrams = self._get_ngrams(hypothesis, n + 1)

                match_count = 0

                for ngram in hypothesis_ngrams:
                    if ngram in reference_ngrams:
                        reference_ngrams.remove(ngram)

                        match_count += 1

                self._match_counts[n] += match_count
                self._counts[n] += len(hypothesis_ngrams)

    def get(self):
        precisions = [sys.float_info.min for n in range(self.N)]

        i = 1

        for n in range(self.N):
            match_counts = self._match_counts[n]
            counts = self._counts[n]

            if counts != 0:
                if match_counts == 0:
                    i *= 2
                    match_counts = 1 / i

                if (match_counts / counts) > 0:
                    precisions[n] = match_counts / counts

        bleu = self._get_brevity_penalty() * math.exp(sum(map(math.log, precisions)) / self.N)

        return (self.name, bleu)

    def calculate(self):
        precisions = [sys.float_info.min for n in range(self.N)]

        i = 1

        for n in range(self.N):
            match_counts = self._match_counts[n]
            counts = self._counts[n]

            if counts != 0:
                if match_counts == 0:
                    i *= 2
                    match_counts = 1 / i

                precisions[n] = match_counts / counts

        return self._get_brevity_penalty() * math.exp(sum(map(math.log, precisions)) / self.N)

    def _get_brevity_penalty(self):
        if self._size_hyp >= self._size_ref:
            return 1
        else:
            if self._size_hyp > 0:
                size_hyp = self._size_hyp
            else:
                size_hyp = 1

            return math.exp(1 - (self._size_ref / size_hyp))

    @staticmethod
    def _get_ngrams(sentence, n):
        ngrams = []

        if len(sentence) >= n:
            for i in range(len(sentence) - n + 1):
                ngrams.append(sentence[i:i+n])

        return ngrams

    @staticmethod
    def _convert(nd_list):
        if len(nd_list) == 0:
            return []

        new_list = [[] for _ in range(nd_list[0].shape[0])]

        for element in nd_list:
            for i in range(element.shape[0]):
                new_list[i].append(element[i].asscalar())

        return new_list

<#if tc.containsAdaNet()>
def objective_function(model, data, loss, gamma=.0000001) -> float:
    """
    :param model:
    :param trainer:
    :param data:
    :param loss:
    :param gamma:
    :return:
    """
    data.reset()
    err_list = []
    for batch_i, batch in enumerate(data):
        pred = model(batch.data[0])
        label = batch.label[0]
        error = loss(pred, label)
        err_list.append(error)
    err = concatenate(err_list)
    c_complexities = model.get_candidate_complexity()
    c_complexities = c_complexities.reshape((1, c_complexities.shape[0])) * gamma

    objective = add(err.mean(), c_complexities)

    return objective[0][0]


def calculate_l1(params: dict) -> float:
    """
    calculate the L1 Norm on the weights of the passed model
    """
    parameter = params
    l1 = None
    for key in parameter:
        if 'weight' in key:
            if l1 is None:
                l1 = parameter[key].data().abs().sum()
            else:
                l1 = add(l1, parameter[key].data().abs().sum())
    return l1


class CandidateTrainingloss(Loss):
    def __init__(self,
                 weight=None,
                 candidate=None,
                 loss=SigmoidBCELoss,
                 batch_axis=0,
                 alpha=.07,
                 beta=0.0001,
                 gamma=.0001,
                 **kwargs):
        """
        loss function which is used to train each candidate

        :param  loss, can be any (binary) loss function, to the result a regularization term is
        added which consists of  complexity of the candidate and the L1-Norm applied
        to the candidate weights
        """
        super(CandidateTrainingloss, self).__init__(weight, batch_axis, **kwargs)
        self.a = alpha
        self.b = beta
        self.g = gamma
        self.coreLoss = loss  # in template, the loss function is passed initialized!!!!
        self.model = candidate  # candidate to be trained

    # noinspection PyMethodOverriding
    def hybrid_forward(self, F, x, label, *args, **kwargs):
        l1 = calculate_l1(self.model.collect_params()) * self.g

        # calculate regularization term reg
        # reg = (alpha*r)+beta , r = rademacher complexity approximation
        rade_term = F.add(F.multiply(self.a, self.model.approximate_rade()), self.b)
        reg_term = F.multiply(rade_term, l1)

        # save the regularization term, since it is needed in the calculation of the objective function
        self.model.update_complexity(reg_term)

        # calculate the actual loss and add the regularization term
        l = self.coreLoss(x, label)
        ad = F.multiply(F.ones(l.shape), reg_term)

        res = F.add(l, ad)

        return res


class AdaLoss(Loss):
    """
    objective function of the whole model
    """

    def __init__(self, weight=None, model=None, loss=SigmoidBCELoss, loss_args=(True,), batch_axis=0, lamb=0.0001,
                 beta=.0001,
                 **kwargs):
        super(AdaLoss, self).__init__(weight, batch_axis, **kwargs)

        self.coreLoss = loss
        self.model = model
        self.c_complexities = self.model.get_candidate_complexity()  # get candidate complexities
        self.lamb = lamb
        self.beta = beta

    def hybrid_forward(self, F, x, label):
        cl = self.coreLoss(x, label)
        l1 = calculate_l1(self.model.out.collect_params())
        reg_term = F.sum(((self.lamb * self.c_complexities) + self.beta) * l1)
        return F.add(cl, reg_term)


def fitComponent(trainIter: mx.io.NDArrayIter, trainer: mx.gluon.Trainer, epochs: int, component: gluon.HybridBlock,
                 loss_class: gluon.loss, loss_params: dict) -> None:
    """
    function trains a component of the generated model.
    expects a compoment, a trainern instance with corresponding parameters.

    """
    loss = loss_class(**loss_params)
    for epoch in range(epochs):
        trainIter.reset()
        for batch_i, batch in enumerate(trainIter):
            with autograd.record():
                data = batch.data[0]
                label = batch.label[0]
                pred = component(data)
                error = loss(pred, label)
            error.backward()
            trainer.step(data.shape[0], ignore_stale_grad=True)


def get_trainer(optimizer: str, parameters: dict, optimizer_params: dict) -> mx.gluon.Trainer:
    # gluon.Trainer doesnt take a ctx
    if optimizer == 'Adamw':
        trainer = mx.gluon.Trainer(parameters, AdamW.AdamW(**optimizer_params), )
    else:
        trainer = mx.gluon.Trainer(parameters, optimizer, optimizer_params)
    return trainer


def fit(loss: gluon.loss.Loss,
        optimizer: str,
        epochs: int,
        optimizer_params: dict,
        dataLoader,
        dataClass,
        shuffle_data: bool,
        preprocessing: bool,
        T=100,
        batch_size=10,
        ctx=None,
        logging=None
        ) -> gluon.HybridBlock:
    logging.info("AdaNet: starting ...")
    cg = dataClass.Builder()
    model_template = dataClass.model_template
    model_operations = {}
    model_score = None
    model = model_template(model_operations)
    if ctx is None:
        ctx = mx.gpu() if mx.context.num_gpus() else mx.cpu()

    model.initialize(ctx=ctx)

    if preprocessing:
        preproc_lib = "CNNPreprocessor_${tc.fileNameWithoutEnding?keep_after("CNNSupervisedTrainer_")}_executor"
        train_iter, test_iter, data_mean, data_std, train_images, test_images = dataLoader.load_preprocessed_data(
            batch_size, preproc_lib, shuffle_data)
    else:
        train_iter, test_iter, data_mean, data_std, train_images, test_images = dataLoader.load_data(batch_size,
                                                                                                     shuffle_data)

    for rnd in range(T):
        # get new candidates
        c0, c1 = cg.get_candidates()
        c0.initialize(ctx=ctx)
        c1.initialize(ctx=ctx)
        c0.hybridize()
        c1.hybridize()

        # train candidate 0
        c0_trainer = get_trainer(optimizer, c0.collect_params(), optimizer_params)
        fitComponent(trainIter=train_iter, trainer=c0_trainer, epochs=epochs, component=c0,
                     loss_class=CandidateTrainingloss, loss_params={'loss': loss, 'candidate': c0})

        # train candidate 1
        c1_trainer = get_trainer(optimizer, c1.collect_params(), optimizer_params)
        fitComponent(trainIter=train_iter, trainer=c1_trainer, epochs=epochs, component=c1,
                     loss_class=CandidateTrainingloss, loss_params={'loss': loss, 'candidate': c1})

        # create model with candidate 0 added -> c0_model
        c0_work_op = model_operations.copy()
        c0_work_op[c0.name] = c0

        c0_model = model_template(operations=c0_work_op)
        c0_model.out.initialize(ctx=ctx)
        if c0_model.finalout:
            c0_model.finalout.initialize(ctx=ctx)
        c0_model.hybridize()

        # create model with candidate 1 added -> c1_model
        c1_work_op = model_operations.copy()
        c1_work_op[c1.name] = c1

        c1_model = model_template(operations=c1_work_op)
        c1_model.out.initialize(ctx=ctx)
        if c1_model.finalout:
            c1_model.finalout.initialize(ctx=ctx)
        c1_model.hybridize()

        # train c0_model
        params = c0_model.out.collect_params()
        params.update(c0_model.finalout.collect_params())
        c0_out_trainer = get_trainer(optimizer, params, optimizer_params)
        fitComponent(trainIter=train_iter, trainer=c0_out_trainer, epochs=epochs, component=c0_model,
                     loss_class=AdaLoss, loss_params={'loss': loss, 'model': c0_model})

        # train c1_model
        params = c1_model.out.collect_params()
        params.update(c1_model.finalout.collect_params())
        c1_out_trainer = get_trainer(optimizer, params, optimizer_params)
        fitComponent(trainIter=train_iter, trainer=c1_out_trainer, epochs=epochs, component=c1_model,
                     loss_class=AdaLoss, loss_params={'loss': loss, 'model': c1_model})

        c0_score = objective_function(model=c1_model, data=train_iter, loss=loss)

        c1_score = objective_function(model=c1_model, data=train_iter, loss=loss)

        check = nd.greater_equal(c0_score, c1_score)

        # decide which candidate yields the best improvement
        model, operation, score = (c0_model, c0, c0_score) if check else (c1_model, c1, c1_score)

        if model_score is None:
            model_score = score
            old_score = nd.array(model_score)
        else:
            # if the new score is better than the old one continue else return current model
            if score <= model_score:
                old_score = nd.array(model_score)
                model_score = nd.array(score)
            else:
                logging.info("AdaNet: abort in Round {}/{}".format(rnd + 1, T))
                # this is not a finally trained model!!
                model = model_template(operations=model_operations)
                model.initialize(ctx=ctx)
                return model

        model_operations[operation.name] = operation
        cg.update()
        round_msg = 'AdaNet:round: {}/{} finished,'.format(rnd + 1, T)
        improvement = (1 - (model_score / old_score).asscalar()) * 100
        score_msg = 'current model score:{:.5f} improvement {:.5f}%'.format(model_score.asscalar(), improvement)
        logging.info(round_msg + score_msg)

    return model

</#if>

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
              preprocessing = False):
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
            train_iter, test_iter, data_mean, data_std, train_images, test_images = self._data_loader.load_data(batch_size, shuffle_data)

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
        elif loss == 'softmax_cross_entropy_ignore_indices':
            fromLogits = loss_params['from_logits'] if 'from_logits' in loss_params else False
            loss_function = SoftmaxCrossEntropyLossIgnoreIndices(axis=loss_axis, ignore_indices=ignore_indices, from_logits=fromLogits, sparse_label=sparseLabel, batch_axis=batch_axis)
        elif loss == 'sigmoid_binary_cross_entropy':
            loss_function = mx.gluon.loss.SigmoidBinaryCrossEntropyLoss()
        elif loss == 'cross_entropy':
            loss_function = CrossEntropyLoss(axis=loss_axis, sparse_label=sparseLabel, batch_axis=batch_axis)
        elif loss == 'dice_loss':
            loss_weight = loss_params['loss_weight'] if 'loss_weight' in loss_params else None
            loss_function = DiceLoss(axis=loss_axis, weight=loss_weight, sparse_label=sparseLabel, batch_axis=batch_axis)
        elif loss == 'softmax_cross_entropy_ignore_label':
            loss_weight = loss_params['loss_weight'] if 'loss_weight' in loss_params else None
            loss_ignore_label = loss_params['loss_ignore_label'] if 'loss_ignore_label' in loss_params else None
            loss_function = SoftmaxCrossEntropyLossIgnoreLabel(axis=loss_axis, ignore_label=loss_ignore_label, weight=loss_weight, batch_axis=batch_axis)
        elif loss == 'l2':
            loss_function = mx.gluon.loss.L2Loss()
        elif loss == 'l1':
            loss_function = mx.gluon.loss.L2Loss()
        elif loss == 'huber':
            rho = loss_params['rho'] if 'rho' in loss_params else 1
            loss_function = mx.gluon.loss.HuberLoss(rho=rho)
        elif loss == 'hinge':
            loss_function = mx.gluon.loss.HingeLoss(margin=margin)
        elif loss == 'squared_hinge':
            loss_function = mx.gluon.loss.SquaredHingeLoss(margin=margin)
        elif loss == 'logistic':
            labelFormat = loss_params['label_format'] if 'label_format' in loss_params else 'signed'
            loss_function = mx.gluon.loss.LogisticLoss(label_format=labelFormat)
        elif loss == 'kullback_leibler':
            fromLogits = loss_params['from_logits'] if 'from_logits' in loss_params else True
            loss_function = mx.gluon.loss.KLDivLoss(from_logits=fromLogits)
        elif loss == 'log_cosh':
            loss_function = LogCoshLoss()
        else:
            logging.error("Invalid loss parameter.")
        loss_function.hybridize()
<#if tc.containsAdaNet()>
<#list tc.architecture.networkInstructions as networkInstruction>

        assert self._networks[${networkInstruction?index}].AdaNet, "passed model is not an AdaNet model"
        self._networks[${networkInstruction?index}] = fit(
                    loss=loss_function,
                    optimizer=optimizer,
                    epochs=num_epoch,
                    optimizer_params = optimizer_params,
                    dataLoader = self._data_loader,
                    dataClass = self._dataClass[${networkInstruction?index}],
                    shuffle_data=shuffle_data,
                    preprocessing=preprocessing,
                    T=100,
                    batch_size=batch_size,
                    ctx=mx_context[0],
                    logging=logging
                )
        logging.info(self._networks[0])
        #put here the AdaNet logic
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
                    train_iter, test_iter, data_mean, data_std, train_images, test_images = self._data_loader.load_data(batch_size, shuffle_data)

            global_loss_train = 0.0
            train_batches = 0

            loss_total = 0
            train_iter.reset()
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
                metric = mx.metric.create(eval_metric, **eval_metric_params)
                for batch_i, batch in enumerate(train_iter):

<#if episodicReplayVisited?? && anyEpisodicLocalAdaptation && !containsUnrollNetwork>
<#include "pythonExecuteTest.ftl">

                        predictions = []
                        for output_name in outputs:
                            if mx.nd.shape_array(mx.nd.squeeze(output_name)).size > 1:
                                predictions.append(mx.nd.argmax(output_name, axis=1))
                            else:
                                predictions.append(output_name)

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
                            predictions.append(mx.nd.argmax(output_name, axis=1))
                        else:
                            predictions.append(output_name)

                    metric.update(preds=predictions, labels=[labels[j] for j in range(len(labels))])
</#if>

                train_metric_score = metric.get()[1]
            else:
                train_metric_score = 0

            global_loss_test = 0.0
            test_batches = 0
    
            test_iter.batch_size = single_pu_batch_size
            test_iter.reset()
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
                                predictions.append(mx.nd.argmax(output_name, axis=1))
                            else:
                                predictions.append(output_name)

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
                        predictions.append(mx.nd.argmax(output_name, axis=1))
                    else:
                        predictions.append(output_name)

                metric.update(preds=predictions, labels=[labels[j] for j in range(len(labels))])

            global_loss_test /= (test_batches * single_pu_batch_size)
</#if>
            test_metric_name = metric.get()[0]
            test_metric_score = metric.get()[1]

            metric_file = open(self._net_creator._model_dir_ + 'metric.txt', 'w')
            metric_file.write(test_metric_name + " " + str(test_metric_score))
            metric_file.close()

            logging.info("Epoch[%d] Train metric: %f, Test metric: %f, Train loss: %f, Test loss: %f" % (epoch, train_metric_score, test_metric_score, global_loss_train, global_loss_test))

            if (epoch+1) % checkpoint_period == 0:
                for i, network in self._networks.items():
                    network.save_parameters(self.parameter_path(i) + '-' + str(epoch).zfill(4) + '.params')
                    if hasattr(network, 'episodic_sub_nets'):
                        for j, net in enumerate(network.episodic_sub_nets):
                            episodic_layers[i][j].save_memory(self.parameter_path(i) + "_episodic_memory_sub_net_" + str(j + 1) + "-" + str(epoch).zfill(4))

        for i, network in self._networks.items():
            network.save_parameters(self.parameter_path(i) + '-' + str((num_epoch-1) + begin_epoch).zfill(4) + '.params')
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
