/* (c) https://github.com/MontiCore/monticore */
//import SchemaWithObjectTypeEntry;

schema Supervised {

    reference-model: referencemodels.ddpg.DDPG2
    network_type: schema {
        gnn;
    }

    //enums
     context: enum {
            cpu, gpu;
        }

    //object type
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

    // primitive types
    eval_metric: eval_metric_type
    batch_size: N1
    num_epoch: N
    checkpoint_period = 5: N
    preprocessing_name: component
    load_pretrained: B
    log_period: N
    shuffle_data: B
}