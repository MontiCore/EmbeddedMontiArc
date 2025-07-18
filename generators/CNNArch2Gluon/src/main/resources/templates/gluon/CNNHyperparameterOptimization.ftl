import logging
from mxnet.numpy import nan
import optuna
import mxnet as mx
from mxnet import gluon, autograd

<#list configurations as config>
class CNNHyperparameterOptimization_${config.instanceName}:
</#list>

    def __init__(self, data_loader, net_constructor):
        self._data_loader = data_loader
        self.train_dataset, self.test_dataset, self.val_dataset = None, None, None
        self.train_iter, self.test_iter, self.val_iter = None, None, None

        self._net_creator = net_constructor
        self._networks = {}

        self.ctx, self.batch_size, self.epochs, self.loss_fct, self.eval_metric = None, None, None, None, None
        self.optimizer_params = None
        self.cleaning, self.cleaning_params = None, None
        self.data_imbalance, self.data_imbalance_params = None, None
        self.data_splitting, self.data_splitting_params = None, None


    def hyperparameter_optimization( 
        self, batch_size=64, num_epoch=10, context='cpu', 
        optimizer=None, optimizer_params=(None), 
        cleaning=None, cleaning_params=(None),
        data_splitting=None, data_splitting_params=(None),
        data_imbalance=None, data_imbalance_params=(None), 
        loss_fct='softmax_cross_entropy', eval_metric='acc',
        dataset=None, test_dataset=None, val_dataset=None 
    ):
        '''
        Hyperparameter Optimization
        - return: trial with best hyperparameters
        '''
        self.save_parameters(
            batch_size, num_epoch, context, optimizer_params, 
            cleaning, cleaning_params,
            data_imbalance, data_imbalance_params, 
            data_splitting, data_splitting_params,
            loss_fct, eval_metric,
            dataset, test_dataset, val_dataset
        )

        # start optimization
        logging.info(" - Start Hyperparameter Tuning - ")
        study = optuna.create_study(direction="maximize",pruner=optuna.pruners.ThresholdPruner(lower=0.2))
        study.optimize(lambda t: self.objective(trial=t), n_trials=self.optimizer_params["ntrials"])

        logging.info(" - Hyperparameter Tuning Done - ")
        logging.info(" Number of finished trials: " + str(len(study.trials)))

        logging.info(" Best trial:")
        trial = study.best_trial

        logging.info(" - Validation Value: " + str(trial.value))

        logging.info(" - Params: ")
        for key, value in trial.params.items():
            logging.info("\t{}: {}".format(key, value))

        self.retrain_model(trial)

        if data_imbalance is not None:
            if data_imbalance_params['check_bias']:
                self._data_loader.check_bias()

        fig = optuna.visualization.plot_optimization_history(study)
        fig.show()


    def objective(self, trial):
        '''
        Define Objective Function. \n
        Optuna is a black-box optimizer, which means it needs an objective function, which returns a numerical value 
        to evaluate the performance of the hyperparameters, and decide where to sample in upcoming trials.
        - return: trial with best hyperparameters
        '''

        ## Candidates to be verified by Optuna.
        # optimizer declaration
        if self.optimizer_params['optimizer_options'] != []:
            optimizer = trial.suggest_categorical("optimizer", self.optimizer_params["optimizer_options"])
        else: 
            logging.error(" Optimizer Options in Network.conf is empty!")
            logging.info(" Using default optimizer: Adam and SGD.")
            optimizer = trial.suggest_categorical("optimizer", ["Adam", "SGD"])
        
        # learning rate declaration
        if self.optimizer_params['learning_rate_range'] != [] and len(self.optimizer_params['learning_rate_range']) == 2:
            lr = trial.suggest_float(
                "learning_rate", 
                self.optimizer_params['learning_rate_range'][0], 
                self.optimizer_params['learning_rate_range'][1], 
                log=True
            )
        else: 
            logging.error(" Learning rate range in Network.conf is either empty or wrongly initialized!")
            logging.info(" Using default range 1e-5 to 1.")
            lr = trial.suggest_float( "learning_rate", 1e-5, 1, log=True)

        # weight decay declaration
        if self.optimizer_params['weight_decay_range'] != [] and len(self.optimizer_params['weight_decay_range']) == 2:
            wd = trial.suggest_float(
                "weight_decay", 
                self.optimizer_params['weight_decay_range'][0], 
                self.optimizer_params['weight_decay_range'][1], 
                log=True
            )
        else: 
            logging.error(" Weight decay range in Network.conf is either empty or wrongly initialized!")
            logging.info(" Using default range 1e-5 to 1e-1.")
            wd = trial.suggest_float( "weight_decay", 1e-5, 1e-1, log=True)

        # momentum declaration
        if optimizer in ["NAG", "SGD"] and self.optimizer_params['momentum_range'] != [] and len(self.optimizer_params['momentum_range']) == 2:
            mom = trial.suggest_float(
                "momentum", 
                self.optimizer_params['momentum_range'][0], 
                self.optimizer_params['momentum_range'][1], 
                log=True
            )
        elif self.optimizer_params['momentum_range'] == [] or len(self.optimizer_params['momentum_range']) != 2:
            logging.error(" Momentum range in Network.conf is either empty or wrongly initialized!")
            logging.info(" Using default range 1e-1 to 9e-1.")
            mom = trial.suggest_float( "momentum", 1e-1, 9e-1, log=True)
        
        # cleaning parameter declaration
        if self.optimizer_params["with_cleaning"]:
            missing = trial.suggest_categorical("clean_missing", [True, False])
            duplicate = trial.suggest_categorical("clean_duplicate", [True, False])
            noisy = trial.suggest_categorical("clean_noisy", [True, False])

        #Definition network, trainer, metric.
        self._net_creator.construct(context=self.ctx, batch_size=self.batch_size)
        self._networks = self._net_creator.networks
        net = self._net_creator.networks[0]
        
        net.initialize(mx.init.Xavier(magnitude=2.24), ctx=self.ctx, force_reinit=True)

        if optimizer in ["NAG", "SGD"]:
            parameters = dict({
                'learning_rate': lr,
                'wd': wd,
                'momentum': mom,
            })
        else: 
            parameters = dict({
                'learning_rate': lr,
                'wd': wd,
            })

        trainer = gluon.Trainer(
            params=net.collect_params(),
            optimizer=optimizer,
            optimizer_params=parameters
        )
        
        # Load Dataset with cleaning params
        if self.optimizer_params["with_cleaning"]:
            cleaning_params = {
                'missing': missing,
                'duplicate': duplicate,
                'noisy': noisy   
            }
        else: cleaning_params=None

        train_iter, _, val_iter = self.reload_data(cleaning_params=cleaning_params)
        
        ctx = mx.gpu() if mx.context.num_gpus() > 0 else mx.cpu(0)

        # Train and validation        
        for epoch in range(self.epochs):
            # start training
            metric_train = mx.metric.create(self.eval_metric)
            train_iter.reset()

            for batch in train_iter:
                with autograd.record():
                    outputs = net(batch.data[0].as_in_context(ctx))[0][0]
                    labels = [batch.label[0].as_in_context(ctx)]

                    loss = self.loss_fct(outputs, batch.label[0].as_in_context(ctx))
                    loss.backward()

                metric_train.update(labels=[labels[j] for j in range(len(labels))], preds=[mx.nd.argmax(out, axis=1) for out in [outputs]])
                trainer.step(batch_size=self.batch_size)

            name, acc = metric_train.get()
            
            # start validation
            metric_val = mx.metric.create(self.eval_metric)
            val_iter.reset()

            for batch in val_iter:
                outputs = net(batch.data[0].as_in_context(ctx))[0][0]
                labels = [batch.label[0].as_in_context(ctx)]
                metric_val.update(labels=[labels[j] for j in range(len(labels))], preds=[mx.nd.argmax(out, axis=1) for out in [outputs]])
            
            _, acc_val = metric_val.get()
            
            logging.info(f" Epoch[{epoch+1}] Training: {name}={format(acc, '.6f')}, Validation: {name}={format(acc_val, '.6f')}")

            # check if trial should be pruned
            trial.report(acc_val, epoch)
            if trial.should_prune():
                logging.error(f" Trial {trial.number} pruned with parameters={trial.params}")
                raise optuna.exceptions.TrialPruned()
            
        logging.info(f" Trial {trial.number} finished with parameters={trial.params} and val_acc={acc_val}")
            
        return acc_val


    def retrain_model(self, best_trial):
        """
            Retrain Model with best hyperparameters from trial, evaluate and saves model.\n
            Input:
            - best_trials
        """
        logging.info("")
        logging.info(" - Start Model Evaluation -")

        #Definition network, trainer, metric.
        net = self._net_creator.networks[0]
        net.initialize(mx.init.Xavier(magnitude=2.24), ctx=self.ctx, force_reinit=True)

        if best_trial.params["optimizer"] in ["NAG", "SGD"]:
            best_parameters=dict({
                'learning_rate': best_trial.params["learning_rate"],
                'wd': best_trial.params["weight_decay"],
                'momentum': best_trial.params["momentum"]
            })
        else:
            best_parameters=dict({
                'learning_rate': best_trial.params["learning_rate"],
                'wd': best_trial.params["weight_decay"]
            })

        trainer = gluon.Trainer(
            params=net.collect_params(),
            optimizer=best_trial.params["optimizer"],
            optimizer_params=best_parameters
        )


        # Load Dataset with cleaning params
        if self.optimizer_params["with_cleaning"]:
            cleaning_params = {
                'missing': best_trial.params["clean_missing"],
                'duplicate': best_trial.params["clean_duplicate"],
                'noisy': best_trial.params["clean_noisy"]   
            }
        else: cleaning_params = None

        train_iter, test_iter, _ = self.reload_data(cleaning_params=cleaning_params)

        ctx = mx.gpu() if mx.context.num_gpus() > 0 else mx.cpu(0)

        # Train and evaluate model        
        for epoch in range(self.epochs):
            # start training
            metric_train = mx.metric.create(self.eval_metric)
            train_iter.reset()
            train_loss, log_loss = 0.0, 0

            for i_batch, batch in enumerate(train_iter):
                with autograd.record():
                    outputs = net(batch.data[0].as_in_context(ctx))[0][0]
                    labels = [batch.label[0].as_in_context(ctx)]

                    loss = self.loss_fct(outputs, batch.label[0].as_in_context(ctx))
                    loss.backward()

                log_loss += loss.sum().asscalar()
                train_loss += loss.sum().asscalar()

                metric_train.update(labels=[labels[j] for j in range(len(labels))], preds=[mx.nd.argmax(out, axis=1) for out in [outputs]])
                trainer.step(batch_size=self.batch_size)

                if i_batch % 100 == 0 and i_batch != 0:
                    logging.info(" Epoch[%d] Batch[%d] Loss: %.5f" % (epoch, i_batch, log_loss / (self.batch_size * 100)))
                    log_loss = 0
            
            train_loss /= ((i_batch+1) * self.batch_size)

            # start evaluation
            metric_test = mx.metric.create(self.eval_metric)
            test_iter.reset()
            test_loss, log_loss = 0.0, 0

            for i_batch, batch in enumerate(test_iter):
                outputs = net(batch.data[0].as_in_context(ctx))[0][0]
                labels = [batch.label[0].as_in_context(ctx)]

                loss = self.loss_fct(outputs, batch.label[0].as_in_context(ctx))

                metric_test.update(labels=[labels[j] for j in range(len(labels))], preds=[mx.nd.argmax(out, axis=1) for out in [outputs]])

                log_loss += loss.sum().asscalar()
                test_loss += loss.sum().asscalar()
            
            test_loss /= ((i_batch+1) * self.batch_size)

            logging.info(
                " Epochs[%d] Train metric: %f, Test metric: %f, Train loss: %f, Test loss: %f" 
                % (epoch, metric_train.get()[1], metric_test.get()[1], train_loss, test_loss)
            )

        # save network
        for i, network in self._networks.items():
            path = str(self._net_creator._model_basedir_) +'/model' + '_' + str(i)           
            network.save_parameters(path + '-' + str((self.epochs-1) + 0).zfill(4) + '.params')
            network.export(path + '_newest', epoch=0)
            try:
                self.loss_fct.export(path + '_newest_loss', epoch=0)
            except RuntimeError:
                logging.info("Forward for loss functions was not run, export is not possible.")


    def reload_data(self, cleaning_params):
        """
            Load train, test and validation dataset. Depending on setting if hyperparameter optimization
            should include cleaning parameters, the dataset is loaded differently. \n
            returns three datasets: `train_iter`, `test_iter` and `val_iter`
        """

        if  self.optimizer_params["with_cleaning"]:
            train_iter, test_iter, val_iter, _, _, _, _, _, _ = self._data_loader.load_data(
                self.batch_size, 'remove', cleaning_params, self.data_imbalance, self.data_imbalance_params, self.data_splitting, self.data_splitting_params, 
                'hpo', False, False, self.train_dataset, self.test_dataset, self.val_dataset
            )
        else:
            if self.train_iter is None and self.test_iter is None and self.val_iter is None:
                train_iter, test_iter, val_iter, _, _, _, _, _, _ = self._data_loader.load_data(
                    self.batch_size, self.cleaning, self.cleaning_params, 
                    self.data_imbalance, self.data_imbalance_params, self.data_splitting, self.data_splitting_params, 
                    'hpo', False, False, self.train_dataset, self.test_dataset, self.val_dataset
                )
                self.train_iter, self.test_iter, self.val_iter = train_iter, test_iter, val_iter
            elif self.train_iter is not None and self.test_iter is not None and self.val_iter is not None:
                return self.train_iter, self.test_iter, self.val_iter
            else:
                logging.error(f"At least one of the datasets empty!")
            
        return train_iter, test_iter, val_iter


    def save_parameters(
        self, batch_size, num_epoch, context, optimizer_params, 
        cleaning, cleaning_params,
        data_imbalance, data_imbalance_params, 
        data_splitting, data_splitting_params,
        loss_fct, eval_metric,
        dataset, test_dataset, val_dataset
    ):
        self.batch_size = batch_size
        self.epochs = num_epoch
        self.eval_metric = eval_metric

        if optimizer_params != (None):
            self.optimizer_params = optimizer_params
        else: logging.error(" No hyperparameter optimization settings specified in Network.conf!")

        self.train_dataset, self.test_dataset, self.val_dataset = dataset, test_dataset, val_dataset
        
        self.cleaning, self.cleaning_params = cleaning, cleaning_params
        self.data_imbalance, self.data_imbalance_params = data_imbalance, data_imbalance_params
        self.data_splitting, self.data_splitting_params = data_splitting, data_splitting_params

        if context == 'gpu' or context == 'cpu':
            self.ctx = [mx.gpu(i) for i in range(mx.context.num_gpus())] if context == 'gpu' else [mx.cpu()]
        else: logging.error(" Context argument is '" + context + "'. Only 'cpu' and 'gpu are valid arguments'.")
        
        if loss_fct == 'softmax_cross_entropy':
            self.loss_fct = mx.gluon.loss.SoftmaxCrossEntropyLoss(axis=-1, from_logits=False, sparse_label=True, batch_axis=0)
        else: # Basic loss
            self.loss_fct = mx.gluon.loss.Loss()
