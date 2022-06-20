# (c) https://github.com/MontiCore/monticore
from abc import ABCMeta, abstractmethod

class AbstractTrainer(metaclass=ABCMeta):
    @abstractmethod
    def train(self):
        pass

class SupervisedTrainer(AbstractTrainer, metaclass=ABCMeta):

    def __init__(self, model, train_config, train_loader, test_loader, model_dir, model_prefix):
        super().__init__()
        self._model_ = model
        self._train_config_ = train_config
        self._train_loader_ = train_loader
        self._test_loader_ = test_loader
        self._model_dir_ = model_dir
        self._model_prefix_ = model_prefix

    def train(self):
        #later we will take this from train_config.loss()
        criterion = nn.CrossEntropyLoss()
        #later we will take this from train_config.optimizer()
        optimizer = torch.optim.SGD(self._model_.parameters(), lr=0.001, momentum=0.9)
        begin_epoch= 0
        self._model_.train()
        num_epoch = 2
        train_losses_total = []
        train_accuracy_total = []
        #later we will take this from train_config.num_epoch()
        for epoch in range(num_epoch):  # loop over the dataset multiple times
            train_losses = []
            train_accuracy = []
            for i, data in enumerate(self._train_loader_, 0):
                # get the inputs; data is a list of [inputs, labels]
                inputs, labels = data
                # zero the parameter gradients
                optimizer.zero_grad()
                # forward + backward + optimize
                outputs = self._model_(inputs)
                loss = criterion(outputs, labels)
                _, predicted = torch.max(outputs.data, 1)
                train_losses.append(loss)
                train_accuracy.append(torch.tensor((torch.sum(predicted == labels).item() / len(predicted))))
                loss.backward()
                optimizer.step()
            train_losses_total.append(torch.stack(train_losses).mean().item())
            train_accuracy_total.append(torch.stack(train_accuracy).mean().item())
            print(f' Epoch: {epoch+1} Train Loss:{(torch.stack(train_losses).mean().item()):.4f} Train Accuracy:{100 *(torch.stack(train_accuracy).mean().item()):.2f}%')
        loss_overall= (torch.tensor(train_losses_total)).mean().item()
        accurracy_overall= (100 * (torch.tensor(train_accuracy)).mean().item())
        print(f'Overall Training Loss and accuracy after  {num_epoch} epochs,  Loss:{((torch.tensor(train_losses_total)).mean().item()):.4f} , Accuracy:{(100 * (torch.tensor(train_accuracy)).mean().item()):.2f}%')
        try:
            os.makedirs(self._model_dir_)
        except OSError:
            if not os.path.isdir(self._model_dir_):
                raise
        torch.save({
                    'epoch': str(num_epoch + begin_epoch),
                    'model_state_dict': self._model_.state_dict(),
                    'optimizer_state_dict': optimizer.state_dict(),
                    'loss': loss}, self._model_dir_ + self._model_prefix_ + "_" + str(num_epoch + begin_epoch))
        #Saving model in .pt format for prediction in c++
        model_scripted = torch.jit.script(self._model_) # Export to TorchScript
        model_scripted.save('model_scripted.pt')
        torch.save(model, PATH)
        return {'trained_model': self._model_ , 'accuracy':accurracy_overall,'loss':loss_overall }
        #print(torch.tensor(train_accuracy_total))