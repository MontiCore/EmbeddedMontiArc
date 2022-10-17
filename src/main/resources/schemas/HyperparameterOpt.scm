/* (c) https://github.com/MontiCore/monticore */

schema HyperparameterOpt {

    optimizer: enum {
        SA, BO, WeightedRS, GA, PSO, Hyperband, BOHB, DEHB;
    }

}