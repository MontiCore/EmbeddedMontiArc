# `SchemaLang`
A textual language that allows defining schemas for [`ConfLang`](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/conflang) configurations.

## Usage

```
schema Example {

}
```

### Parameter Declarations

#### Enumerated Parameters

```
schema Example {

    enum_parameter: enum {
        first_value,
        second_value;
    }
}
```

#### Typed Parameters
```
schema Example {

    integer_parameter: Z
    rational_parameter: Q
    string_parameter: string
    component_parameter: component
}
```

##### Supported Types

| Type          | Description                              |
| :-----------: |------------------------------------------|
| `N`           | Natural numbers, including `0`           |
| `N1`          | Natural numbers, excluding `0`           |
| `Z`           | Whole numbers, i.e. integers             |
| `Q`           | Rational numbers                         |
| `C`           | Complex numbers                          |
| `B`           | Boolean                                  |
| `string`      | String                                   |
| `component`   | Fully-qualified references to components |
| Object type   | See next [section](#objecttypes)         |


`SchemaLang` also allows the definition of ranges for all numeric types. Ranges can be of four different types, which are exemplarily shown in the following table for the type `Q`.

| Type                | Description                  |
| :-----------------: |------------------------------|
| `Q[min:max]`  | Closed interval, i.e. includes `min` and `max` values |
| `Q(min:max)`  | Open interval, i.e. excludes `min` and `max` values |
| `Q(min:max]`  | Left-open interval, i.e. excludes `min` and includes `max` values |
| `Q[min:max)`  | Right-open interval, i.e. includes `min` and excludes `max` values |

Ranges can also be used with a `scale`. For example, `N(0:2:8)` would only allow the values `{2, 4, 6}`.


###  <a name="objecttypes"></a> Object Types
Nested parameters in configurations can be restricted with object type definitions.

```
schema Example {

    object_parameter: object_type
    
    object_type {
        // List of allowed values for the nested parameter.
        values: sgd, adam;
    
        // Parameters that are allowed for all above values.
        learning_rate: Q

        // Parameters that are only allowed if the value is sgd.
        define sgd {
            momentum: Q
        }
    }
}
```

## Example
```
schema Example {

    enum_parameter: enum {
        first_value,
        second_value;
    }

    integer_parameter: Z
    integer_parameter_with_range: Z(-10:10)
    integer_property_with_range_and_scale: Z(-10:2:10)
    
    string_parameter: string
    component_parameter: component
    object_parameter: object_type
    
    object_type {
    }
}
```