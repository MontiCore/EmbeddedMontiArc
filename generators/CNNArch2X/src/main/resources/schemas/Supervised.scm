/* (c) https://github.com/MontiCore/monticore */
import EvalMetric;
import Loss;

schema Supervised extends General {

    batch_size: N1
    num_epoch: N1
    normalize: B
    checkpoint_period = 5: N
    preprocessing_name: component
    load_checkpoint: B
    load_pretrained: B
    log_period: N
    eval_train: B
    use_teacher_forcing: B
    save_attention_image: B
    eval_metric: eval_metric_type
    loss: loss_type
}