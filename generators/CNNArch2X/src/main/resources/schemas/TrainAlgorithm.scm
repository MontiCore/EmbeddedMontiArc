/* (c) https://github.com/MontiCore/monticore */

schema TrainAlgorithm {

    save_trained_architecture: B
    architecture_save_path: string
    train_algorithm_name: enum {
        EfficientNet, AdaNet;
    }
    train_pipeline_name: enum {
        Pytorch, Gluon, Tensorflow;
    }

}