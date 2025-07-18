<!-- (c) https://github.com/MontiCore/monticore -->
# EMAM2RosCpp
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2RosCpp/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2RosCpp/badges/master/coverage.svg)

## Warning
This generator is part of an composite generator and does not create an executable. Look at [EMAM2Middleware](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware) if you want to generate one.

## How does the new array handling work?

To give an example of the improved array handling:
 
 `tag test.array with RosConnection = { topic = (/test/input, nav_msgs/Path), msgField=data[5:9].position.orientation.x }`
 
There are two new features that can be used regarding arrays in EMAM:

   1.  A range can be defined, from which the **input** values will be taken
        --> in this example the array named "array" from the component "test" will get the input values from the topic /test/input on indices: 5 to 9. 
        All other values in "array" (if it contains more than 5 values) will be filled with zeros.
        
   2.  Here "/test/input" is an array of structs (e.g poses). It is now possible to use the message field to get values out of structs (like in many other programming languages)
        by using "dots" like in:  `struct.attributes`