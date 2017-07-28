# Documentation of EmbeddedMontiArcMath

This is only a short outline, please change it if it does not fit to your tutorial.
And please add additional points, if you need need them.
Every point should present exactly one feature and it should also contain minimal (but complete - model should be parseable) code examples (several code examples per points are possible, e.g showing different units).

EmbeddedMontiArc (Yannick)
----
* Introduction/Motivation.
* How to create a component?
* How to add ports to a component?
* How to instantiate a component?
* How to connect components?
* How to deal with component and port arrays?
* How to deal with generics and default generic values?
* How to deal with configuration parameters?

Math (Sascha)
----
* How to create a matrix?
* What Types of matrices are possible?
* How to deal with units? (compatiblity, conversion of units)
* How to select values from a matrix?
* Listing of all supported opertions (+,*, ^, \, .*, .\, ...)
* Listing of all supported Octave functions (eig, diag, ...)

EmbeddedMontiArcMath (Sascha)
----
* How to embedded Math language into EmbeddedMontiArc?
* How ports of EmbeddedMontiArc are adapted to matrices into Math language?

EmbeddedMontiView (Fabian)
----
* What is the difference between EmbeddedMontiArc and EmbeddedMontiView?
* How to model abstract ports (with no name, with no data type and both)?
* What is an textual anonymous port ($port1)? 
* What are abstract connectors?
* What are abstract hierarchies?
* What are abstract effectors?

OCL (Ferdinand)
----
* How to define an invariant?
* What does context mean?
* How to use quantificators?
* How to deal with Lists, Sets?
* How to deal with implications, and other operators?
* What are OCL pre- and post conditions?
