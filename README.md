# `ConfLang`

`ConfLang` is a general configuration language.

## Usage
Every configuration starts with the `configuration` keyword followed by the name of the configuration
and curly brackets. The name must be camel-case.

```
configuration MyConfiguration {
}
```

Every configuration can have an optional `package` declaration.

```
package de.monticore.lang.conflang.examples;

configuration MyConfiguration {
}
```

A configuration can have an arbitrary number of simple or nested configuration entries. A *simple* configuration entry is a configuration entry with a single atomic configuration value.

`ConfLang` is a typed configuration language supporting the following value types.

| Type          | Description   | Examples                           |
| ------------- |---------------| -----------------------------------|
| Null          | Null value.   | `null`                             |
| Char          | A character parenthesized with single quotes.     | `'c'`                              |
| String        | Some character sequence parenthesized with double quotes. | `"This is a string value"`         |
| Boolean       | Boolean values "true" and "false".      | `true`, `false`                    |
| Number        | Includes signed and unsigned integer, long, float, and double values. | `5`, `-17`, `-0.00000197` |
| Vector        |       | `(-152)`, `("ConfLang", 258.9765)`, `(-152, 2.5, 0.00001, "ConfLang")` |

## Examples
The following listing shows an exemplary `ConfLang` configuration instance for a  supervised training scenario in the domain of machine learning.

```
package de.monticore.lang.conflang.examples;

configuration SupervisedLearning {
    num_epoch = 5000
    batch_size = 100
    load_checkpoint = true
    eval_metric = "mse"
    loss = softmax_cross_entropy {
        sparse_label = true
        from_logits = true
        loss_axis = -1
        batch_axis = 0
    }
    context = "gpu"
    normalize = true
    optimizer = rmsprop {
        learning_rate = 0.001
        learning_rate_minimum = 0.00001
        weight_decay = 0.01
        learning_rate_decay = 0.9
        learning_rate_policy = "step"
        step_size = 1000
        rescale_grad = 1.1
        clip_gradient = 10
        gamma1 = 0.9
        gamma2 = 0.9
        epsilon = 0.000001
        centered = true
        clip_weights = 10
    }
}
```
