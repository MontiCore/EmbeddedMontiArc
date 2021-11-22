import os
import shutil
import sys
import warnings

import mxnet as mx
import logging
import time
import numpy as np
import matplotlib.pyplot as plt
from mxnet import nd, autograd, gluon
from mxnet.gluon import nn

class VAELoss(gluon.loss.Loss):
    def __init__(self, weight=None, batch_axis=0, batch_size=1, kl_loss_weight=1, loss_ctx_list=[], model_ctx=mx.cpu(), **kwargs):
        super(VAELoss, self).__init__(weight,batch_axis,**kwargs)
        self.soft_zero = 1e-10
        self.batch_size = batch_size
        self.kl_weight = kl_loss_weight
        self.model_ctx = model_ctx
        self.loss_ctx_list = loss_ctx_list

    def hybrid_forward(self, F, pred, data, loss_param_list=None):
        loss = 0
        if len(self.loss_ctx_list) != 0:
            for i in range(len(self.loss_ctx_list)):
                dict = self.loss_ctx_list[i]
                loss_params = loss_param_list[i]
                val_dict = dict["values"]
                if dict["loss"] == "kl_div_loss":
                    pdf = val_dict["pdf"]
                    loss = - self.kl_loss(F, pdf, loss_params, self.kl_weight)
                elif dict["loss"] == "quantization_loss":
                    encoding = loss_params[0]
                    quantization = loss_params[1]
                    commitment_weight = val_dict['beta']

                    # Calculate vector quantization loss
                    commitment_loss = self.commitment_weight * F.mean((F.stop_gradient(quantization) - encoding) ** 2, axis=0, exclude=True)
                    codebook_loss = F.mean((quantization - F.stop_gradient(encoding)) ** 2, axis=0, exclude=True)
                    loss = commitment_loss + codebook_loss

        bce = - F.sum(data * F.log(pred + self.soft_zero) + (1 - data) * F.log(1 - pred + self.soft_zero), axis=0, exclude=True)
        loss = bce + loss
        return loss

    def kl_loss(self, F, pdf,  params, weight=1):
        kl = 0

        if pdf == "normal":
            mu = params[0]
            logvar = params[1]
            kl = 0.5 * F.sum(1 + logvar - mu * mu - F.exp(logvar), axis=0, exclude=True)

        return kl

<#assign networkInstruction = tc.architecture.networkInstructions[0]>
<#if tc.architecture.getAuxiliaryArchitecture()??>
<#assign auxNetworkInstruction = tc.architecture.getAuxiliaryArchitecture().networkInstructions[0]>
</#if>

class CNNAutoencoderTrainer:
    def __init__(self, data_loader, encoder_net_constructor, decoder_net_constructor):
        self._data_loader = data_loader
        self._enc_creator = encoder_net_constructor
        self._dec_creator = decoder_net_constructor

    def train(self, batch_size=64,
              num_epoch=10,
              optimizer='adam',
              optimizer_params=(('learning_rate', 0.001),),
              load_checkpoint=False,
              context='cpu',
              checkpoint_period=5,
              load_pretrained=False,
              normalize=False,
              loss='',
              log_period = 50,
              labeled_training=False,
              kl_loss_weight=1,
              print_images=False):
        num_pus = 1
        if context == 'gpu': #(Multi-) GPU training on VAEs not tested yet
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
        single_pu_batch_size = int(batch_size / num_pus)

        if print_images:
            try:
                os.path.isdir('images')
            except:
                logging.info("Creating 'images' directory.")
                try:
                    os.mkdir('images')
                except OSError:
                    logging.error("Creation of the 'images' directory failed.")

        if preprocessing:
            preproc_lib = "CNNPreprocessor_${tc.fileNameWithoutEnding?keep_after("CNNAutoencoderTrainer_")}_executor"
            train_iter, test_iter, data_mean, data_std, _, _ = self._data_loader.load_preprocessed_vae_data(batch_size=batch_size, preproc_lib=preproc_lib, use_labels=labeled_training)
        else:
            train_iter, test_iter, data_mean, data_std, _, _ = self._data_loader.load_vae_data(batch_size=batch_size,use_labels=labeled_training)

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
            self._enc_creator.construct(context=mx_context, data_mean=data_mean, data_std=data_std)
            self._dec_creator.construct(context=mx_context, data_mean=data_mean, data_std=data_std)
        else:
            self._enc_creator.construct(context=mx_context)
            self._dec_creator.construct(context=mx_context)

        begin_epoch = 0
        if load_checkpoint:
            begin_epoch = self._enc_creator.load(mx_context)
        elif load_pretrained:
            self._enc_creator.load_pretrained_weights(mx_context)
            self._dec_creator.load_pretrained_weights(mx_context)
        else:
            if os.path.isdir(self._enc_creator._model_dir_):
                shutil.rmtree(self._enc_creator._model_dir_)
            if os.path.isdir(self._dec_creator._model_dir_):
                shutil.rmtree(self._dec_creator._model_dir_)

        encoder_nets = self._enc_creator.networks
        decoder_nets = self._dec_creator.networks

        loss_ctx_list = []

        <#if networkInstruction.body.hasLossParameterizingElements() >loss_ctx_list.append(encoder_nets[0].loss_ctx_dict)</#if>
        <#if tc.architecture.getAuxiliaryArchitecture()??>
        <#if auxNetworkInstruction.body.hasLossParameterizingElements() >loss_ctx_list.append(decoder_nets[0].loss_ctx_dict)</#if>
        </#if>


        enc_trainers = [mx.gluon.Trainer(network.collect_params(), optimizer, optimizer_params) for network in
                    encoder_nets.values() if len(network.collect_params().values()) != 0]
        dec_trainers = [mx.gluon.Trainer(network.collect_params(), optimizer, optimizer_params) for network in
                    decoder_nets.values() if len(network.collect_params().values()) != 0]

        if loss == "customloss":
            try:
                import customloss
                loss_function = customloss.Loss(batch_size=batch_size, kl_loss_weight=kl_loss_weight, loss_ctx_list=loss_ctx_list)
            except ImportError:
                logging.info("No customloss directory found. Using default loss function instead.")
                loss_function = VAELoss(batch_size=batch_size, kl_loss_weight=kl_loss_weight, loss_ctx_list=loss_ctx_list)
        else:
            loss_function = VAELoss(batch_size=batch_size, kl_loss_weight=kl_loss_weight, loss_ctx_list=loss_ctx_list)

        loss_function.hybridize()

        tic = None

        avg_speed = 0
        n = 0

        train_lost_list = []
        test_lost_list = []

        for epoch in range(begin_epoch, begin_epoch + num_epoch):
            global_loss_train = 0.0
            train_batches = 0

            loss_total = 0
            train_iter.reset()

            for batch_i, batch in enumerate(train_iter):

                with autograd.record():

                    <#if tc.architectureInputs??>
                    ${tc.architectureInputs[0]} = gluon.utils.split_and_load(batch.data[0], ctx_list=mx_context, even_split=False)
                    </#if>

                    lossList = []
                    loss_param_list = []
                    encoding_ = []
                    pred_ = []

                    for i in range(num_pus):
                        lossList.append([])
                        loss_param_list.append([])
                        encoding_.append([])
                        pred_.append([])

                    if labeled_training:
                        <#if tc.architectureInputs[1]??>
                        ${tc.architectureInputs[1]} = gluon.utils.split_and_load(batch.label[0], ctx_list=mx_context, even_split=False)

                        labelsoh = [mx.nd.one_hot(${tc.architectureInputs[1]}[i],${tc.architectureInputSymbols[1].ioDeclaration.type.dimensions[0]?c}) for i in range(num_pus)]
                        </#if>
                        <#--<#list tc.architecture.networkInstructions as networkInstruction> -->
                        nd.waitall()
                        for i in range(num_pus):
                            feature_vec<#if networkInstruction.body.hasLossParameterizingElements() >, loss_params_enc</#if> = encoder_nets[0](data_[i],labelsoh[i]) <#--${networkInstruction?index} -->
                            <#if networkInstruction.body.hasLossParameterizingElements() >loss_param_list[i].append(loss_params_enc)</#if>
                            encoding_[i] = feature_vec[0]<#if networkInstruction.body.hasLossParameterizingElements()><#else>[0]</#if>

                        <#if tc.architecture.getAuxiliaryArchitecture()??>
                        <#-- <#list tc.architecture.getAuxiliaryArchitecture().networkInstructions as networkInstruction> -->
                        nd.waitall()
                        for i in range(num_pus):
                            res_<#if auxNetworkInstruction.body.hasLossParameterizingElements() >, loss_params_dec</#if> = decoder_nets[0](encoding_[i],labelsoh[i]) <#--${networkInstruction?index} -->
                            <#if auxNetworkInstruction.body.hasLossParameterizingElements() >loss_param_list[i].append(loss_params_dec)</#if>
                        <#--</#list> -->
                            pred_[i] = res_[0]<#if auxNetworkInstruction.body.hasLossParameterizingElements()><#else>[0]</#if>
                        </#if>
                    else:
                        nd.waitall()
                        for i in range(num_pus):
                            feature_vec<#if networkInstruction.body.hasLossParameterizingElements() >, loss_params_enc</#if> = encoder_nets[0](data_[i]) <#--${networkInstruction?index} -->
                            <#if networkInstruction.body.hasLossParameterizingElements() >loss_param_list.append(loss_params_enc)</#if>
                            encoding_[i] = feature_vec[0]<#if networkInstruction.body.hasLossParameterizingElements()><#else>[0]</#if>
                        <#--</#list> -->

                        <#if tc.architecture.getAuxiliaryArchitecture()??>
                        <#-- <#list tc.architecture.getAuxiliaryArchitecture().networkInstructions as networkInstruction> -->
                        nd.waitall()
                        for i in range(num_pus):
                            res_<#if auxNetworkInstruction.body.hasLossParameterizingElements() >, loss_params_dec</#if> = decoder_nets[0](encoding_[i]) <#--${networkInstruction?index} -->
                            <#if auxNetworkInstruction.body.hasLossParameterizingElements() >loss_param_list[i].append(loss_params_dec)</#if>
                            pred_[i] = res_[0]<#if auxNetworkInstruction.body.hasLossParameterizingElements()><#else>[0]</#if>
                        </#if>
                    [lossList[i].append(loss_function(pred_[i], data_[i], loss_param_list[i])) for i in range(num_pus)]


                    losses = [0] * num_pus
                    for i in range(num_pus):
                        for element in lossList[i]:
                            losses[i] = losses[i] + element

                for loss in losses:
                    loss.backward()
                    loss_total += loss.sum().asscalar()
                    global_loss_train += loss.sum().asscalar()

                train_batches += 1

                for trainer in dec_trainers:
                    trainer.step(batch_size)
                for trainer in enc_trainers:
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

                        logging.info("Epoch[%d] Batch[%d] Speed: %.2f samples/sec Average Loss: %.5f" % (
                        epoch, batch_i, speed, loss_avg))

                        avg_speed += speed
                        n += 1

                        tic = time.time()


            global_loss_train /= (train_batches * batch_size)

            tic = None

            global_loss_test = 0.0
            test_batches = 0

            test_iter.batch_size = single_pu_batch_size
            test_iter.reset()


            for batch_i, batch in enumerate(test_iter):

                loss_param_list = []

                <#if tc.architectureInputs??>
                ${tc.architectureInputs[0]} = gluon.utils.split_and_load(batch.data[0], ctx_list=mx_context, even_split=False)
                </#if>

                lossList = []
                loss_param_list = []
                encoding_ = []
                pred_ = []

                for i in range(num_pus):
                    lossList.append([])
                    loss_param_list.append([])
                    encoding_.append([])
                    pred_.append([])

                if labeled_training:
                    <#if tc.architectureInputs[1]??>
                    ${tc.architectureInputs[1]} = gluon.utils.split_and_load(batch.label[0], ctx_list=mx_context, even_split=False)

                    labelsoh = [mx.nd.one_hot(${tc.architectureInputs[1]}[i],${tc.architectureInputSymbols[1].ioDeclaration.type.dimensions[0]?c}) for i in range(num_pus)]
                    </#if>
                    <#--<#list tc.architecture.networkInstructions as networkInstruction> -->
                    nd.waitall()
                    for i in range(num_pus):
                        feature_vec<#if networkInstruction.body.hasLossParameterizingElements() >, loss_params_enc</#if> = encoder_nets[0](data_[i],labelsoh[i]) <#--${networkInstruction?index} -->
                        <#if networkInstruction.body.hasLossParameterizingElements() >loss_param_list[i].append(loss_params_enc)</#if>
                        encoding_[i] = feature_vec[0]<#if networkInstruction.body.hasLossParameterizingElements()><#else>[0]</#if>

                    <#if tc.architecture.getAuxiliaryArchitecture()??>
                    <#-- <#list tc.architecture.getAuxiliaryArchitecture().networkInstructions as networkInstruction> -->
                    nd.waitall()
                    for i in range(num_pus):
                        res_<#if auxNetworkInstruction.body.hasLossParameterizingElements() >, loss_params_dec</#if> = decoder_nets[0](encoding_[i],labelsoh[i]) <#--${networkInstruction?index} -->
                        <#if auxNetworkInstruction.body.hasLossParameterizingElements() >loss_param_list[i].append(loss_params_dec)</#if>
                        <#--</#list> -->
                        pred_[i] = res_[0]<#if auxNetworkInstruction.body.hasLossParameterizingElements()><#else>[0]</#if>
                    </#if>
                else:
                    nd.waitall()
                    for i in range(num_pus):
                        feature_vec<#if networkInstruction.body.hasLossParameterizingElements() >, loss_params_enc</#if> = encoder_nets[0](data_[i]) <#--${networkInstruction?index} -->
                        <#if networkInstruction.body.hasLossParameterizingElements() >loss_param_list.append(loss_params_enc)</#if>
                        encoding_[i] = feature_vec[0]<#if networkInstruction.body.hasLossParameterizingElements()><#else>[0]</#if>
                    <#--</#list> -->

                    <#if tc.architecture.getAuxiliaryArchitecture()??>
                    <#-- <#list tc.architecture.getAuxiliaryArchitecture().networkInstructions as networkInstruction> -->
                    nd.waitall()
                    for i in range(num_pus):
                        res_<#if auxNetworkInstruction.body.hasLossParameterizingElements() >, loss_params_dec</#if> = decoder_nets[0](encoding_[i]) <#--${networkInstruction?index} -->
                        <#if auxNetworkInstruction.body.hasLossParameterizingElements() >loss_param_list[i].append(loss_params_dec)</#if>
                        pred_[i] = res_[0]<#if auxNetworkInstruction.body.hasLossParameterizingElements()><#else>[0]</#if>
                    </#if>
                [lossList[i].append(loss_function(pred_[i], data_[i], loss_param_list[i])) for i in range(num_pus)]

                losses = [0] * num_pus
                for i in range(num_pus):
                    for element in lossList[i]:
                        losses[i] = losses[i] + element

                for loss in losses:
                    global_loss_test += loss.sum().asscalar()

                global_loss_test += loss.sum().asscalar()

                test_batches += 1

            global_loss_test /= (test_batches * single_pu_batch_size)

            logging.info("Epoch[%d], Train loss: %f, Validation loss: %f" % (
                epoch, global_loss_train, global_loss_test))

            if (epoch+1) % checkpoint_period == 0:
                for i, network in encoder_nets.items():
                    network.save_parameters(self.parameter_path(i) + '-' + str(epoch).zfill(4) + '.params')
                for i, network in decoder_nets.items():
                    network.save_parameters(self.parameter_path(i) + '-' + str(epoch).zfill(4) + '.params')

            if print_images:
                train_lost_list.append(global_loss_train)
                test_lost_list.append(global_loss_test)

                #Reconstructions
                filename = 'test_reconstruction_%06d%06d.png' % (epoch, batch_i)
                fig = plt.figure()
                ax = fig.add_subplot(1, 2, 1)
                plt.imshow(data_[0][0].squeeze(0).asnumpy())
                ax.set_title('Original')
                ax = fig.add_subplot(1, 2, 2)
                plt.imshow(pred_[0][0].squeeze(0).asnumpy())
                ax.set_title('Reconstruction')
                plt.tight_layout()
                plt.savefig('images/' + filename)
                plt.close()

        if print_images:
            #Loss plot
            batch_x = np.linspace(1, num_epoch, len(train_lost_list))
            filename = 'loss_graph.png'
            plt.plot(batch_x, np.array(train_lost_list))
            plt.plot(batch_x, np.array(test_lost_list))
            plt.legend(['Train loss', 'Validation Loss'])
            plt.savefig('images/' + filename)

        for i, network in encoder_nets.items():
            network.save_parameters(self.encoder_parameter_path(i) + '-' + str((num_epoch-1) + begin_epoch).zfill(4) + '.params')
            network.export(self.encoder_parameter_path(i) + '_newest', epoch=0)

        for i, network in decoder_nets.items():
            network.save_parameters(self.decoder_parameter_path(i) + '-' + str((num_epoch-1) + begin_epoch).zfill(4) + '.params')
            network.export(self.decoder_parameter_path(i) + '_newest', epoch=0)

    def encoder_parameter_path(self, index):
        return self._enc_creator._model_dir_ + self._enc_creator._model_prefix_ + '_' + str(index)

    def decoder_parameter_path(self, index):
        return self._dec_creator._model_dir_ + self._dec_creator._model_prefix_ + '_' + str(index)