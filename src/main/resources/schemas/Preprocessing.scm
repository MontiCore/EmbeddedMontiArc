/* (c) https://github.com/MontiCore/monticore */

schema Preprocessing {

    train_split: Q
    norm_method: {
        normalization, standardization;
    }
    grayscale: B
    data_augmentation: B

}