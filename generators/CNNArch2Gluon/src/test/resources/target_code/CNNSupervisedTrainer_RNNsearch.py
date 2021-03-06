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
            label = gluon.loss._reshape_like(F, label, pred)
            loss = -(pred * label).sum(axis=self._axis, keepdims=True)
        # ignore some indices for loss, e.g. <pad> tokens in NLP applications
        for i in self._ignore_indices:
            loss = loss * mx.nd.logical_not(mx.nd.equal(mx.nd.argmax(pred, axis=1), mx.nd.ones_like(mx.nd.argmax(pred, axis=1))*i) * mx.nd.equal(mx.nd.argmax(pred, axis=1), label))
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



class CNNSupervisedTrainer_RNNsearch:
    def __init__(self, data_loader, net_constructor):
        self._data_loader = data_loader
        self._net_creator = net_constructor
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
              log_period=50,
              context='gpu',
              save_attention_image=False,
              use_teacher_forcing=False,
              normalize=True,
              preprocessing = False):
        if context == 'gpu':
            mx_context = mx.gpu()
        elif context == 'cpu':
            mx_context = mx.cpu()
        else:
            logging.error("Context argument is '" + context + "'. Only 'cpu' and 'gpu are valid arguments'.")

        if preprocessing:
            preproc_lib = "CNNPreprocessor_RNNsearch_executor"
            train_iter, test_iter, data_mean, data_std, train_images, test_images = self._data_loader.load_preprocessed_data(batch_size, preproc_lib)
        else:
            train_iter, test_iter, data_mean, data_std, train_images, test_images = self._data_loader.load_data(batch_size)

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
        else:
            if os.path.isdir(self._net_creator._model_dir_):
                shutil.rmtree(self._net_creator._model_dir_)

        self._networks = self._net_creator.networks

        try:
            os.makedirs(self._net_creator._model_dir_)
        except OSError:
            if not os.path.isdir(self._net_creator._model_dir_):
                raise

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

        tic = None

        for epoch in range(begin_epoch, begin_epoch + num_epoch):

            loss_total = 0
            train_iter.reset()
            for batch_i, batch in enumerate(train_iter):
                with autograd.record():
                    labels = [batch.label[i].as_in_context(mx_context) for i in range(30)]

                    source_ = batch.data[0].as_in_context(mx_context)

                    target_ = [mx.nd.zeros((batch_size, 1,), ctx=mx_context) for i in range(30)]

                    encoder_state_ = mx.nd.zeros((batch_size, 2, 1000,), ctx=mx_context)
                    encoder_output_ = mx.nd.zeros((batch_size, 30, 2000,), ctx=mx_context)
                    fc_output_ = mx.nd.zeros((batch_size, 30, 1000,), ctx=mx_context)
                    decoder_state_ = mx.nd.zeros((batch_size, 1, 1000,), ctx=mx_context)
                    decoder_output_ = mx.nd.zeros((batch_size, 1, 1000,), ctx=mx_context)

                    const1_ = mx.nd.full((batch_size, 1,), 1, ctx=mx_context)

                    nd.waitall()

                    lossList = []

                    fc_output_, encoder_state_, encoder_output_ = self._networks[0](source_, encoder_state_)

                    target_[0] = self._networks[1](const1_)

                    lossList.append(loss_function(target_[0], labels[0]))
                    decoder_state_ = self._networks[2](encoder_state_)

                    for i in range(1, 30):
                        target_[i-1+1], decoder_state_, decoder_output_ = self._networks[3](decoder_state_, fc_output_, target_[i-1+0])

                        lossList.append(loss_function(target_[i-1+1], labels[i-1+1]))
                        target_[i-1+1] = mx.nd.argmax(target_[i-1+1], axis=1).expand_dims(1)
                        if use_teacher_forcing == "True":
                            target_[i-1+1] = mx.nd.expand_dims(labels[i-1+1], axis=1)

                    loss = 0
                    for element in lossList:
                        loss = loss + element

                loss.backward()

                loss_total += loss.sum().asscalar()

                for trainer in trainers:
                    trainer.step(batch_size)

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

                        tic = time.time()

            tic = None


            if eval_train:
                train_iter.reset()
                metric = mx.metric.create(eval_metric, **eval_metric_params)
                for batch_i, batch in enumerate(train_iter):
                    labels = [batch.label[i].as_in_context(mx_context) for i in range(30)]

                    source_ = batch.data[0].as_in_context(mx_context)

                    target_ = [mx.nd.zeros((batch_size, 1,), ctx=mx_context) for i in range(30)]

                    encoder_state_ = mx.nd.zeros((batch_size, 2, 1000,), ctx=mx_context)
                    encoder_output_ = mx.nd.zeros((batch_size, 30, 2000,), ctx=mx_context)
                    fc_output_ = mx.nd.zeros((batch_size, 30, 1000,), ctx=mx_context)
                    decoder_state_ = mx.nd.zeros((batch_size, 1, 1000,), ctx=mx_context)
                    decoder_output_ = mx.nd.zeros((batch_size, 1, 1000,), ctx=mx_context)

                    const1_ = mx.nd.full((batch_size, 1,), 1, ctx=mx_context)

                    nd.waitall()

                    outputs = []
                    attentionList=[]
                    fc_output_, encoder_state_, encoder_output_ = self._networks[0](source_, encoder_state_)

                    target_[0] = self._networks[1](const1_)

                    outputs.append(target_[0])
                    decoder_state_ = self._networks[2](encoder_state_)

                    k = 3
                    sequences = [([target_[1-1+0]], mx.nd.full((batch_size, 1,), 1.0, ctx=mx_context), [mx.nd.full((batch_size, 64,), 0.0, ctx=mx_context)])]

                    for i in range(1, 30):
                        all_candidates = []

                        for seq, score, attention in sequences:
                            target_[i-1+0] = seq[-1]
                            target_[i-1+1], decoder_state_, decoder_output_ = self._networks[3](decoder_state_, fc_output_, target_[i-1+0])
                            out = target_[i-1+1]

                            topk = out.topk(k=k)

                            for top_index in range(len(topk[0])):
                                j = mx.nd.slice_axis(topk, axis=1, begin=top_index, end=top_index+1)
                                currentScore = mx.nd.slice_axis(out, axis=1, begin=top_index, end=top_index+1)
                                newScore = mx.nd.expand_dims(score.squeeze() * currentScore.squeeze(), axis=1)
                                candidate = (seq + [j],  newScore, attention + [])
                                all_candidates.append(candidate)

                        ordered = []
                        newSequences = []
                        for batch_entry in range(batch_size):
                            ordered.append([])
                            batchCandidate = [([seq[batch_entry] for seq in candidate[0]], candidate[1][batch_entry], [attention[batch_entry].expand_dims(axis=0) for attention in candidate[2]]) for candidate in all_candidates]
                            ordered[batch_entry] = sorted(batchCandidate, key=lambda tup: tup[1].asscalar())
                            if batch_entry == 0:
                                newSequences = ordered[batch_entry]
                            else:
                                newSequences = [([mx.nd.concat(newSequences[sequenceIndex][0][seqIndex], ordered[batch_entry][sequenceIndex][0][seqIndex], dim=0) for seqIndex in range(len(newSequences[sequenceIndex][0]))],
                                    mx.nd.concat(newSequences[sequenceIndex][1], ordered[batch_entry][sequenceIndex][1], dim=0),
                                    [mx.nd.concat(newSequences[sequenceIndex][2][attentionIndex], ordered[batch_entry][sequenceIndex][2][attentionIndex], dim=0) for attentionIndex in range(len(newSequences[sequenceIndex][2]))])
                                    for sequenceIndex in range(len(newSequences))]

                        newSequences = [([newSequences[sequenceIndex][0][seqIndex].expand_dims(axis=1) for seqIndex in range(len(newSequences[sequenceIndex][0]))],
                            newSequences[sequenceIndex][1].expand_dims(axis=1), [newSequences[sequenceIndex][2][attentionIndex] for attentionIndex in range(len(newSequences[sequenceIndex][2]))])
                            for sequenceIndex in range(len(newSequences))]

                        sequences = newSequences[:][:k]

                    for i in range(1, 30):
                        target_[i-1+1] = sequences[0][0][i]
                        outputs.append(target_[i-1+1])


                    if save_attention_image == "True":
                        import matplotlib
                        matplotlib.use('Agg')
                        import matplotlib.pyplot as plt
                        logging.getLogger('matplotlib').setLevel(logging.ERROR)

                        if(os.path.isfile('src/test/resources/training_data/Show_attend_tell/dict.pkl')):
                            with open('src/test/resources/training_data/Show_attend_tell/dict.pkl', 'rb') as f:
                                dict = pickle.load(f)

                        plt.clf()
                        fig = plt.figure(figsize=(15,15))
                        max_length = len(labels)-1

                        ax = fig.add_subplot(max_length//3, max_length//4, 1)
                        ax.imshow(train_images[0+batch_size*(batch_i)].transpose(1,2,0))

                        for l in range(max_length):
                            attention = attentionList[l]
                            attention = mx.nd.slice_axis(attention, axis=0, begin=0, end=1).squeeze()
                            attention_resized = np.resize(attention.asnumpy(), (8, 8))
                            ax = fig.add_subplot(max_length//3, max_length//4, l+2)
                            if int(labels[l+1][0].asscalar()) > len(dict):
                                ax.set_title("<unk>")
                            elif dict[int(labels[l+1][0].asscalar())] == "<end>":
                                ax.set_title(".")
                                img = ax.imshow(train_images[0+batch_size*(batch_i)].transpose(1,2,0))
                                ax.imshow(attention_resized, cmap='gray', alpha=0.6, extent=img.get_extent())
                                break
                            else:
                                ax.set_title(dict[int(labels[l+1][0].asscalar())])
                            img = ax.imshow(train_images[0+batch_size*(batch_i)].transpose(1,2,0))
                            ax.imshow(attention_resized, cmap='gray', alpha=0.6, extent=img.get_extent())

                        plt.tight_layout()
                        target_dir = 'target/attention_images'
                        if not os.path.exists(target_dir):
                            os.makedirs(target_dir)
                        plt.savefig(target_dir + '/attention_train.png')
                        plt.close()

                    predictions = []
                    for output_name in outputs:
                        if mx.nd.shape_array(mx.nd.squeeze(output_name)).size > 1:
                            predictions.append(mx.nd.argmax(output_name, axis=1))
                        else:
                            predictions.append(output_name)

                    metric.update(preds=predictions, labels=labels)
                train_metric_score = metric.get()[1]
            else:
                train_metric_score = 0

            test_iter.reset()
            metric = mx.metric.create(eval_metric, **eval_metric_params)
            for batch_i, batch in enumerate(test_iter):
                if True:
                    labels = [batch.label[i].as_in_context(mx_context) for i in range(30)]

                    source_ = batch.data[0].as_in_context(mx_context)

                    target_ = [mx.nd.zeros((batch_size, 1,), ctx=mx_context) for i in range(30)]

                    encoder_state_ = mx.nd.zeros((batch_size, 2, 1000,), ctx=mx_context)
                    encoder_output_ = mx.nd.zeros((batch_size, 30, 2000,), ctx=mx_context)
                    fc_output_ = mx.nd.zeros((batch_size, 30, 1000,), ctx=mx_context)
                    decoder_state_ = mx.nd.zeros((batch_size, 1, 1000,), ctx=mx_context)
                    decoder_output_ = mx.nd.zeros((batch_size, 1, 1000,), ctx=mx_context)

                    const1_ = mx.nd.full((batch_size, 1,), 1, ctx=mx_context)

                    nd.waitall()

                    outputs = []
                    attentionList=[]
                    fc_output_, encoder_state_, encoder_output_ = self._networks[0](source_, encoder_state_)

                    target_[0] = self._networks[1](const1_)

                    outputs.append(target_[0])
                    decoder_state_ = self._networks[2](encoder_state_)

                    k = 3
                    sequences = [([target_[1-1+0]], mx.nd.full((batch_size, 1,), 1.0, ctx=mx_context), [mx.nd.full((batch_size, 64,), 0.0, ctx=mx_context)])]

                    for i in range(1, 30):
                        all_candidates = []

                        for seq, score, attention in sequences:
                            target_[i-1+0] = seq[-1]
                            target_[i-1+1], decoder_state_, decoder_output_ = self._networks[3](decoder_state_, fc_output_, target_[i-1+0])
                            out = target_[i-1+1]

                            topk = out.topk(k=k)

                            for top_index in range(len(topk[0])):
                                j = mx.nd.slice_axis(topk, axis=1, begin=top_index, end=top_index+1)
                                currentScore = mx.nd.slice_axis(out, axis=1, begin=top_index, end=top_index+1)
                                newScore = mx.nd.expand_dims(score.squeeze() * currentScore.squeeze(), axis=1)
                                candidate = (seq + [j],  newScore, attention + [])
                                all_candidates.append(candidate)

                        ordered = []
                        newSequences = []
                        for batch_entry in range(batch_size):
                            ordered.append([])
                            batchCandidate = [([seq[batch_entry] for seq in candidate[0]], candidate[1][batch_entry], [attention[batch_entry].expand_dims(axis=0) for attention in candidate[2]]) for candidate in all_candidates]
                            ordered[batch_entry] = sorted(batchCandidate, key=lambda tup: tup[1].asscalar())
                            if batch_entry == 0:
                                newSequences = ordered[batch_entry]
                            else:
                                newSequences = [([mx.nd.concat(newSequences[sequenceIndex][0][seqIndex], ordered[batch_entry][sequenceIndex][0][seqIndex], dim=0) for seqIndex in range(len(newSequences[sequenceIndex][0]))],
                                    mx.nd.concat(newSequences[sequenceIndex][1], ordered[batch_entry][sequenceIndex][1], dim=0),
                                    [mx.nd.concat(newSequences[sequenceIndex][2][attentionIndex], ordered[batch_entry][sequenceIndex][2][attentionIndex], dim=0) for attentionIndex in range(len(newSequences[sequenceIndex][2]))])
                                    for sequenceIndex in range(len(newSequences))]

                        newSequences = [([newSequences[sequenceIndex][0][seqIndex].expand_dims(axis=1) for seqIndex in range(len(newSequences[sequenceIndex][0]))],
                            newSequences[sequenceIndex][1].expand_dims(axis=1), [newSequences[sequenceIndex][2][attentionIndex] for attentionIndex in range(len(newSequences[sequenceIndex][2]))])
                            for sequenceIndex in range(len(newSequences))]

                        sequences = newSequences[:][:k]

                    for i in range(1, 30):
                        target_[i-1+1] = sequences[0][0][i]
                        outputs.append(target_[i-1+1])


                    if save_attention_image == "True":
                        if not eval_train:
                            import matplotlib
                            matplotlib.use('Agg')
                            import matplotlib.pyplot as plt
                            logging.getLogger('matplotlib').setLevel(logging.ERROR)

                            if(os.path.isfile('src/test/resources/training_data/Show_attend_tell/dict.pkl')):
                                with open('src/test/resources/training_data/Show_attend_tell/dict.pkl', 'rb') as f:
                                    dict = pickle.load(f)

                        plt.clf()
                        fig = plt.figure(figsize=(15,15))
                        max_length = len(labels)-1

                        ax = fig.add_subplot(max_length//3, max_length//4, 1)
                        ax.imshow(test_images[0+batch_size*(batch_i)].transpose(1,2,0))

                        for l in range(max_length):
                            attention = attentionList[l]
                            attention = mx.nd.slice_axis(attention, axis=0, begin=0, end=1).squeeze()
                            attention_resized = np.resize(attention.asnumpy(), (8, 8))
                            ax = fig.add_subplot(max_length//3, max_length//4, l+2)
                            if int(mx.nd.slice_axis(outputs[l+1], axis=0, begin=0, end=1).squeeze().asscalar()) > len(dict):
                                ax.set_title("<unk>")
                            elif dict[int(mx.nd.slice_axis(outputs[l+1], axis=0, begin=0, end=1).squeeze().asscalar())] == "<end>":
                                ax.set_title(".")
                                img = ax.imshow(test_images[0+batch_size*(batch_i)].transpose(1,2,0))
                                ax.imshow(attention_resized, cmap='gray', alpha=0.6, extent=img.get_extent())
                                break
                            else:
                                ax.set_title(dict[int(mx.nd.slice_axis(outputs[l+1], axis=0, begin=0, end=1).squeeze().asscalar())])
                            img = ax.imshow(test_images[0+batch_size*(batch_i)].transpose(1,2,0))
                            ax.imshow(attention_resized, cmap='gray', alpha=0.6, extent=img.get_extent())

                        plt.tight_layout()
                        target_dir = 'target/attention_images'
                        if not os.path.exists(target_dir):
                            os.makedirs(target_dir)
                        plt.savefig(target_dir + '/attention_test.png')
                        plt.close()

                predictions = []
                for output_name in outputs:
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
