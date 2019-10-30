import mxnet as mx
import logging
import numpy as np
import time
import os
import shutil
import pickle
import math
import sys
from mxnet import gluon, autograd, nd

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
            label = _reshape_like(F, label, pred)
            loss = -(pred * label).sum(axis=self._axis, keepdims=True)
        #loss = _apply_weighting(F, loss, self._weight, sample_weight)
        # ignore some indices for loss, e.g. <pad> tokens in NLP applications
        for i in self._ignore_indices:
            loss = loss * mx.nd.logical_not(mx.nd.equal(mx.nd.argmax(pred, axis=1), mx.nd.ones_like(mx.nd.argmax(pred, axis=1))*i))
        return loss.mean(axis=self._batch_axis, exclude=True)

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
            return math.exp(1 - (self._size_ref / self._size_hyp))

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



class CNNSupervisedTrainer_CifarClassifierNetwork:
    def applyBeamSearch(input, length, width, maxLength, currProb, netIndex, bestOutput):
        bestProb = 0.0
        while length < maxLength:
            length += 1
            batchIndex = 0
            for batchEntry in input:
                top_k_indices = mx.nd.topk(batchEntry, axis=0, k=width)
                top_k_values = mx.nd.topk(batchEntry, ret_typ='value', axis=0, k=width)
                for index in range(top_k_indices.size):

                    #print mx.nd.array(top_k_indices[index])
                    #print top_k_values[index]
                    if length == 1:
                        #print mx.nd.array(top_k_indices[index])
                        result = applyBeamSearch(self._networks[netIndex](mx.nd.array(top_k_indices[index])), length, width, maxLength,
                            currProb * top_k_values[index], netIndex, self._networks[netIndex](mx.nd.array(top_k_indices[index])))
                    else:
                        result = applyBeamSearch(self._networks[netIndex](mx.nd.array(top_k_indices[index])), length, width, maxLength,
                            currProb * top_k_values[index], netIndex, bestOutput)

                    if length == maxLength:
                        #print currProb
                        if currProb > bestProb:
                            bestProb = currProb
                            bestOutput[batchIndex] = result[batchIndex]
                            #print "new bestOutput: ", bestOutput

                batchIndex += 1
        #print bestOutput
        #print bestProb
        return bestOutput


    def __init__(self, data_loader, net_constructor):
        self._data_loader = data_loader
        self._net_creator = net_constructor
        self._networks = {}

    def train(self, batch_size=64,
              num_epoch=10,
              eval_metric='acc',
              eval_metric_params={},
              loss ='softmax_cross_entropy',
              loss_params={},
              optimizer='adam',
              optimizer_params=(('learning_rate', 0.001),),
              load_checkpoint=True,
              context='gpu',
              checkpoint_period=5,
              normalize=True):
        if context == 'gpu':
            mx_context = mx.gpu()
        elif context == 'cpu':
            mx_context = mx.cpu()
        else:
            logging.error("Context argument is '" + context + "'. Only 'cpu' and 'gpu are valid arguments'.")

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


        train_iter, test_iter, data_mean, data_std = self._data_loader.load_data(batch_size)

        if normalize:
            self._net_creator.construct(context=mx_context, data_mean=data_mean, data_std=data_std)
        else:
            self._net_creator.construct(context=mx_context)

        begin_epoch = 0
        if load_checkpoint:
            begin_epoch = self._net_creator.load(mx_context)
        else:
            if os.path.isdir(self._net_creator._model_dir_):
                shutil.rmtree(self._net_creator._model_dir_)

        self._networks = self._net_creator.networks

        try:
            os.makedirs(self._net_creator._model_dir_)
        except OSError:
            if not os.path.isdir(self._net_creator._model_dir_):
                raise

        trainers = [mx.gluon.Trainer(network.collect_params(), optimizer, optimizer_params) for network in self._networks.values()]

        margin = loss_params['margin'] if 'margin' in loss_params else 1.0
        sparseLabel = loss_params['sparse_label'] if 'sparse_label' in loss_params else True
        if loss == 'softmax_cross_entropy':
            fromLogits = loss_params['from_logits'] if 'from_logits' in loss_params else False
            loss_function = mx.gluon.loss.SoftmaxCrossEntropyLoss(from_logits=fromLogits, sparse_label=sparseLabel)
        elif loss == 'sigmoid_binary_cross_entropy':
            loss_function = mx.gluon.loss.SigmoidBinaryCrossEntropyLoss()
        elif loss == 'cross_entropy':
            loss_function = CrossEntropyLoss(sparse_label=sparseLabel)
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

        speed_period = 50
        tic = None

        for epoch in range(begin_epoch, begin_epoch + num_epoch):
            train_iter.reset()
            for batch_i, batch in enumerate(train_iter):
                data_ = batch.data[0].as_in_context(mx_context)
                softmax_label = batch.label[0].as_in_context(mx_context)

                outputs=[]

                with autograd.record():
                    softmax_ = mx.nd.zeros((batch_size, 10,), ctx=mx_context)

                    lossList = []
                    softmax_ = self._networks[0](data_)
                    lossList.append(loss_function(softmax_, softmax_label))

                    loss = 0
                    for element in lossList:
                        loss = loss + element

                loss.backward()

                for trainer in trainers:
                    trainer.step(batch_size)

                if tic is None:
                    tic = time.time()
                else:
                    if batch_i % speed_period == 0:
                        try:
                            speed = speed_period * batch_size / (time.time() - tic)
                        except ZeroDivisionError:
                            speed = float("inf")

                        logging.info("Epoch[%d] Batch[%d] Speed: %.2f samples/sec" % (epoch, batch_i, speed))

                        tic = time.time()

            tic = None

            train_iter.reset()
            metric = mx.metric.create(eval_metric, **eval_metric_params)
            for batch_i, batch in enumerate(train_iter):
                data_ = batch.data[0].as_in_context(mx_context)

                labels = [
                    batch.label[0].as_in_context(mx_context)
                ]

                outputs=[]

                if True:
                    softmax_ = mx.nd.zeros((batch_size, 10,), ctx=mx_context)

                    softmax_ = self._networks[0](data_)
                    outputs.append(softmax_)

                predictions = []
                for output_name in outputs:
                    if mx.nd.shape_array(mx.nd.squeeze(output_name)).size > 1:
                        predictions.append(mx.nd.argmax(output_name, axis=1))
                    #ArgMax already applied
                    else:
                        predictions.append(output_name)

                '''
                #Compute BLEU and NIST Score if data folder contains a dictionary -> NLP dataset
                if(os.path.isfile('src/test/resources/training_data/Show_attend_tell/dict.pkl')):
                    with open('src/test/resources/training_data/Show_attend_tell/dict.pkl', 'rb') as f:
                        dict = pickle.load(f)

                    import nltk.translate.bleu_score
                    import nltk.translate.nist_score

                    prediction = []
                    for index in range(batch_size):
                        sentence = ''
                        for entry in predictions:
                            sentence += dict[int(entry[index].asscalar())] + ' '
                        prediction.append(sentence)

                    for index in range(batch_size):
                        sentence = ''
                        for batchEntry in batch.label:
                            sentence += dict[int(batchEntry[index].asscalar())] + ' '
                        print("############################")
                        print("label1: ", sentence)
                        print("prediction1: ", prediction[index])

                        BLEUscore = nltk.translate.bleu_score.sentence_bleu([sentence], prediction[index])
                        NISTscore = nltk.translate.nist_score.sentence_nist([sentence], prediction[index])
                        print("BLEU: ", BLEUscore)
                        print("NIST: ", NISTscore)
                        print("############################")
                '''

                metric.update(preds=predictions, labels=labels)
            train_metric_score = metric.get()[1]

            test_iter.reset()
            metric = mx.metric.create(eval_metric, **eval_metric_params)
            for batch_i, batch in enumerate(test_iter):
                data_ = batch.data[0].as_in_context(mx_context)

                labels = [
                    batch.label[0].as_in_context(mx_context)
                ]

                outputs=[]

                if True:
                    softmax_ = mx.nd.zeros((batch_size, 10,), ctx=mx_context)

                    softmax_ = self._networks[0](data_)
                    outputs.append(softmax_)

                predictions = []
                for output_name in outputs:
                    if mx.nd.shape_array(mx.nd.squeeze(output_name)).size > 1:
                        predictions.append(mx.nd.argmax(output_name, axis=1))
                    #ArgMax already applied
                    else:
                        predictions.append(output_name)

                metric.update(preds=predictions, labels=labels)
            test_metric_score = metric.get()[1]

            logging.info("Epoch[%d] Train: %f, Test: %f" % (epoch, train_metric_score, test_metric_score))


            if (epoch - begin_epoch) % checkpoint_period == 0:
                for i, network in self._networks.items():
                    network.save_parameters(self.parameter_path(i) + '-' + str(epoch).zfill(4) + '.params')

        for i, network in self._networks.items():
            network.save_parameters(self.parameter_path(i) + '-' + str(num_epoch + begin_epoch).zfill(4) + '.params')
            network.export(self.parameter_path(i) + '_newest', epoch=0)

    def parameter_path(self, index):
        return self._net_creator._model_dir_ + self._net_creator._model_prefix_ + '_' + str(index)