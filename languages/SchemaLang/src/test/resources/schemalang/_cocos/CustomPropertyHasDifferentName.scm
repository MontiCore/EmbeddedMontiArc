/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema ComplexDefinitionExists {

    eval_metric: eval_metric_object

    eval_metric_object {
        values:
            accuracy,
            cross_entropy,
            f1,
            mae,
            mse,
            perplexity,
            rmse,
            top_k_accuracy,
            accuracy_ignore_label,
            bleu;

        define accuracy_ignore_label {
            axis: Z
            metric_ignore_label: Z
        }

        define bleu {
            exclude: Z*
        }
    }
}