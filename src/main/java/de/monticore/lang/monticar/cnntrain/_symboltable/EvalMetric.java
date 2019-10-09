/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

public enum EvalMetric {
    ACCURACY{
        @Override
        public String toString() {
            return "accuracy";
        }
    },
    BLEU{
        @Override
        public String toString() {
            return "bleu";
        }
    },
    CROSS_ENTROPY{
        @Override
        public String toString() {
            return "crossEntropy";
        }
    },
    F1{
        @Override
        public String toString() {
            return "f1";
        }
    },
    MAE{
        @Override
        public String toString() {
            return "mae";
        }
    },
    MSE{
        @Override
        public String toString() {
            return "mse";
        }
    },
    PERPLEXITY{
        @Override
        public String toString() {
            return "perplexity";
        }
    },
    RMSE{
        @Override
        public String toString() {
            return "rmse";
        }
    },
    TOP_K_ACCURACY{
        @Override
        public String toString() {
            return "topKAccuracy";
        }
    }
}
