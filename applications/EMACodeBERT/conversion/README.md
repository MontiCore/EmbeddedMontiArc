# Conversion of CodeBERT to Gluon
One of the primary problems is that CodeBERT doesn't have any pre-trained models in Gluon. This is necessary to be able to use it with Julian's LoadNetwork layer. Currently CodeBERT has pre-trained models in huggingface, which appears to use the PyTorch format for saving models. So in this directory we have scripts and resources for converting the PyTorch/huggingface pre-trained model to be compatible with gluon and Julian's LoadNetwork layer.

