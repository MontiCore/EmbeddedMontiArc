[![Maintainability](https://api.codeclimate.com/v1/badges/c9ee58c9b0fe15f380f5/maintainability)](https://codeclimate.com/github/EmbeddedMontiArc/CNNTrainLang/maintainability)
[![Build Status](https://travis-ci.org/EmbeddedMontiArc/CNNTrainLang.svg?branch=updated_cnntrain)](https://travis-ci.org/EmbeddedMontiArc/CNNTrainLang)
[![Build Status](https://circleci.com/gh/EmbeddedMontiArc/CNNTrainLang/tree/updated_cnntrain.svg?style=shield&circle-token=:circle-token)](https://circleci.com/gh/EmbeddedMontiArc/CNNTrainLang/tree/updated_cnntrain)
[![Coverage Status](https://coveralls.io/repos/github/EmbeddedMontiArc/CNNTrainLang/badge.svg?branch=updated_cnntrain)](https://coveralls.io/github/EmbeddedMontiArc/CNNTrainLang?branch=updated_cnntrain)

# CNNTrain

CNNTrain is a domain specific language for describing training parameters of a feedforward neural network.
 
CNNTrain files must have .cnnt extension. Training configuration starts with a `configuration` word, followed by the configuration name and a list of parameters.  The available parameters are batch size, number of epochs, loading previous checkpoint as well as an optimizer with its parameters. All these parameters are optional.

An example of a config:
```
configuration FullConfig{
     num_epoch : 5
     batch_size : 100
     load_checkpoint: true
     optimizer:rmsprop{
         learning_rate:0.001
         weight_decay:0.01
         learning_rate_decay:0.9
         learning_rate_policy:step
         step_size:1000
         rescale_grad:1.1
         clip_gradient:10
         gamma1:0.9
         gamma2:0.9
         epsilon:0.000001
         centered:true
         clip_weights:10
     }
}
```
See CNNTrain.mc4 for full grammar definition.

Using CNNTrainGenerator class, a Python file can be generated, which looks as following (for an example above):
 ```python
batch_size = 100,
num_epoch = 5,
load_checkpoint = True,
optimizer = 'rmsprop',
optimizer_params = {
    'epsilon': 1.0E-6,
    'weight_decay': 0.01,
    'rescale_grad': 1.1,
    'centered': True,
    'clip_gradient': 10.0,
    'gamma2': 0.9,
    'gamma1': 0.9,
    'learning_rate_policy': 'step',
    'clip_weights': 10.0,
    'learning_rate': 0.001,
    'learning_rate_decay': 0.9,
    'step_size': 1000}
 ```
To execute generation in your project, use the following code to generate a separate Config file:
```java
import de.monticore.lang.monticar.cnntrain.generator.CNNTrainGenerator;
...
CNNTrainGenerator cnnTrainGenerator =  new CNNTrainGenerator();
Path modelPath = Paths.get("path/to/cnnt/file");
cnnTrainGenerator.generate(modelPath, cnnt_filename);
```

Use the following code to get file contents as a map ( `fileContents.getValue()` contains the generated code):
```java
import de.monticore.lang.monticar.cnntrain.generator.CNNTrainGenerator;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainLanguage;
...
CNNTrainGenerator cnnTrainGenerator =  new CNNTrainGenerator();
ModelPath mp = new ModelPath(Paths.get("path/to/cnnt/file"));
GlobalScope trainScope = new GlobalScope(mp, new CNNTrainLanguage());
Map.Entry<String, String> fileContents = cnnTrainGenerator.generateFileContent( trainScope, cnnt_filename );
        
```
 
 CNNTrain can be used together with [CNNArch](https://github.com/EmbeddedMontiArc/CNNArchLang) language, which describes architecture of a NN. 
 [EmbeddedMontiArcDL](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcDL) uses both languages. 
 
 
