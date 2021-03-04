# CodeBERT Model Conversion
The folder `original_scripts` contains the unmodifed scripts that were used or adapted for the conversion. The folder `modified scripts` contains these adapted scripts and others used for experimentation.

If you are only interested in the final script for the huggingface CodeBERT to Gluon conversion the script is under `modified_scripts/working_attempt/convert_codebert_transformers.py`

The first conversion scripts for experimentation purposes were taken from https://nlp.gluon.ai/model_zoo/conversion_tools/index.html, these were not used in the final script.

The two scripts in `original_scripts/working_attempt` that heavily influenced the final script were taken from the [GluonNLP GitHub](https://github.com/dmlc/gluon-nlp/blob/2bc75db7ae64f8fbb077c08f3b44bc93898090d8/scripts/conversion_toolkits/convert_fairseq_roberta.py) and [huggingface GitHub](https://github.com/huggingface/transformers/blob/b399c26446ea833ab6e9f968fce6fa95c97c6906/src/transformers/convert_roberta_original_pytorch_checkpoint_to_pytorch.py).

Failed experimental attempts can be found under `modified_scripts/initial_attempt` for informational purposes.
