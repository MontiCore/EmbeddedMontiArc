/* (c) https://github.com/MontiCore/monticore */

schema SchemaWithObjectTypeEntry{

    eval_metric_type {
            values:
                rmse,
                accuracy_ignore_label,
                bleu;
             //batchy: N1
            define accuracy_ignore_label {
                axis: Z
                metric_ignore_label: Z
            }

            define bleu {
                exclude: Z*
            }
        }

}