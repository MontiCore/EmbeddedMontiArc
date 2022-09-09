/* (c) https://github.com/MontiCore/monticore */
import Optimizer;
import NoiseDistribution;

schema GAN extends General {

    reference-model: referencemodels.gan.GAN, referencemodels.gan.InfoGAN
    batch_size: N1
    num_epoch: N1
    normalize: B
    checkpoint_period = 5: N
    preprocessing_name: component
    load_checkpoint: B
    load_pretrained: B
    log_period: N
    k_value: N
    generator_target_name: string
    noise_input: string
    generator_loss_weight: Q
    discriminator_loss_weight: Q
    print_images = false: B
    loss_weights: Q*

    discriminator_optimizer: optimizer_type
    noise_distribution: noise_distribution_type
    generator_loss: generator_loss_type

    generator_loss_type {
        values: l1, l2;
    }

    // Conditionally required hyperparameters for training with GAN
    noise_input requires noise_distribution
    generator_loss requires generator_target_name
    generator_target_name requires generator_loss
}