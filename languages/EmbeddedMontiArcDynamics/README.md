<!-- (c) https://github.com/MontiCore/monticore -->
## Dynamic Extensions for EmbeddedMontiArc

## Examples

### EMAD

```
component NotAdapter {
    ports 
        dynamic in B in1[0:32],
        dynamic out B out1[0:32];
       
    instance Not notInstance[0:32];
     
    @ in1::connect && out1::connect {
    
        connect in1[?] -> notInstance[?].in1;
        connect notInstance[?].out1 -> out1[?];
    }
    
}
```

```
component Not {
    ports 
        in B in1,
        out B out1;
    
    @ in1::value( false ) {
        connect true -> out1;
    }
    @ in1::value( true ){
        connect false -> out1;
    }
}
```


### Event

Language for defining events
```
event Test_Value<Port<B> portA>(B val) {

    portA::value( val )

}
```



Define an event for a specific component
```
event Test_Seq for PortValueEMA {

    testValue::value( [42, 123, 456, 789] )
    
}
```
