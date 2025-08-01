/* (c) https://github.com/MontiCore/monticore */

import Loss;

schema Supervised extends WithoutLinkingParameters {

    batch_size: N1
    num_epoch: N1
    load_checkpoint: B
    checkpoint_period: N
    load_pretrained: B
    log_period: N
    eval_train: B
    normalize: B
    use_teacher_forcing: B
    save_attention_image: B
    preprocessing_name: component
    eval_metric: eval_metric
    loss: loss

    eval_metric {
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