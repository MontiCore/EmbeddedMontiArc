# `ConfLang`

`ConfLang` is a general-purpose configuration language with a syntax similar to that of `JSON`.

## Usage
A configuration in `ConfLang` starts with the keyword `configuration` followed by an arbitrary name of the configuration. The name of the configuration must start with a
capital letter. Furthermore, a configuration may have an optional `package` declaration.

For example, the following listing illustrates an empty configuration with the name `LeNet` in package `de.monticore.lang.conflang.examples`.

```
package de.monticore.lang.conflang.examples;

configuration LeNet {
}
```

A configuration may contain an unbounded number of configuration entries, which can be of two different kinds. A *simple* configuration entry is a configuration entry with a single atomic configuration value.

- _Basic name-value configuration entry_: Basic form of a configuration entry consisting
of a name and an atomic value, where the name is an arbitrary continuous string, i.e.
a string without any spaces, and a value.
- _Nested name-value configuration entry_: Extension of former by an additional scope
that might contain arbitrarily many configuration entries of both kinds.

For example, the following configuration contains a basic configuration entry named `num_epoch` with a positive integer value `100`. Also, the configuration contains a nested configuration entry named `optimizer` and value `sgd` with additional subparameters `learning_rate` and `momentum` both with floating-point values.

```
package de.monticore.lang.conflang.examples;

configuration LeNet {
    num_epoch: 100
    optimizer: sgd {
        learning_rate: 0.01
        momentum: 0.9
    }
}
```

The atomic values of configuration entries can be of various types, which are summarized in the table below.

| Literal       | Description   | Examples                           |
| ------------- |---------------| -----------------------------------|
| Character     | A character parenthesized with single quotes     | `'c'`                              |
| String        | Some character sequence parenthesized with double quotes | `"This is a string value"`         |
| Name          | Ordinary name | `supervised`, `ddpg`, `sgd`         |
| Qualified name| Ordinary qualified name | `de.rwth.conflang.Component`         |
| Boolean       | Boolean values "true" and "false"      | `true`, `false`                    |
| Number        | Includes signed and unsigned integer, long, float, and double values | `5`, `-17`, `-0.00000197` |
| List          | List of literals of a fixed length | `(-152)`, `("ConfLang", 258.9765)`, `(-152, 2.5, 0.00001, "ConfLang")` |