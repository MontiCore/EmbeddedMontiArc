/* (c) https://github.com/MontiCore/monticore */
import Optimizer;

schema VAE extends General {

  reference-model: referencemodels.vae.VAE, referencemodels.vae.CVAE
  batch_size: N1
  num_epoch: N1
  normalize: B
  checkpoint_period = 5: N
  load_checkpoint: B
  load_pretrained: B
  log_period: N
  reconstruction_loss = mse: reconLoss_type
  print_images = false: B
  kl_loss_weight: Q

  reconLoss_type {
     values:
         bce,
         mse;
   }

}