<!-- (c) https://github.com/MontiCore/monticore -->
# Documentation of EmbeddedMontiArcMath 

* [Repository Status and Overview](RepoStatus.md)
* [Development Suite for EmbeddedMontiArc](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio)

> Welcome at EmbeddedMontiArc - a development environment and tooling framework for modeling, simulating, and testing the logical layer of embedded and cyber-physical systems. Our base concept is similar to Simulink and Modelica, but with focus on modeling the logical layer and abstracting away from technical details¹ and providing powerful reuse concepts such as generic and configuration parameters²  as well as supporting arrays of components to avoid copying components several times.
> 
> ¹e.g. specify ranges in the physical domain instead of fixed integers: `(0 km/h : 0.1 km/h : 250 km/h)`<br/>
> ²e.g. for library components such as PID controllers

Publications
----
* **[KRRvW17] E. Kusmenko, A. Roth, B. Rumpe, M. von Wenckstern:
  [Modeling Architectures of Cyber Physical Systems](http://www.se-rwth.de/publications/Modeling-Architectures-of-Cyber-Physical-Systems.pdf).
  In: Modelling Foundations and Applications (ECMFA’17),
      Held as Part of STAF 2017, pages 34-50.
      Springer International Publishing, 2017.** (Main Publication for EmbeddedMontiArc's concrete syntax)
* [HKK+18] S. Hillemacher, S. Kriebel, E. Kusmenko, M. Lorang, B. Rumpe, A. Sema, G. Strobl, M. von Wenckstern:
[Model-Based Development of Self-Adaptive Autonomous Vehicles using the SMARDT Methodology](http://www.se-rwth.de/publications/Model-Based-Development-of-Self-Adaptive-Autonomous-Vehicles-using-the-SMARDT-Methodology.pdf).
In: Proceedings of the 6th International Conference on Model-Driven Engineering and Software Development (MODELSWARD'18), pg. 163-178. SciTePress, Jan. 2018.
* [GKR+17] F. Grazioli, E. Kusmenko, A. Roth, B. Rumpe, M. von Wenckstern:
[Simulation Framework for Executing Component and Connector Models of Self-Driving Vehicles](http://www.se-rwth.de/publications/Simulation-Framework-for-Executing-Component-and-Connector-Models-of-Self-Driving-Vehicles.pdf).
In: Proceedings of MODELS 2017. Workshop EXE, Austin, CEUR 2019, Sept. 2017.

Further Examples
----
* **[Collection of over 500 EmbeddedMontiArc/Math Models](https://embeddedmontiarc.github.io/reporting/report/report.html)**
* [Cooperative Driving Examples](examples/cooperativeDriving.md)
* [Racing Car Examples](examples/racingCar/racingCar.md)
* [Topological Mapping Example](examples/toplogicalMapping.md)

This is only a short outline, please change it if it does not fit to your tutorial.
And please add additional points, if you need need them.
Every point should present exactly one feature and it should also contain minimal (but complete - model should be parseable) code examples (several code examples per points are possible, e.g showing different units).

EmbeddedMontiArc (Yannick)
----
* Introduction/Motivation.  

EmbeddedMontiArc (EMA) is a C&C modeling language based on MontiArc. It is the core language of the MontiCAR modeling family where it is used as an architecture description language (ADL) for cyber-physical systems. EmbeddedMontiArc extends MontiArc with the concepts of port and component arrays and introduces a new type system that provides unit support for physical quantities (including possibly limited resolution of the sensors). This liberates the developers from unit checks and conversions, because it enables them to work with real-life  quantities in a type-safe manner. The following describes how to create simple components, add ports and create connections. After that we go into port arrays, generics and parametrization.

* How to create a component?  

A component type can easily be defined by creating a new ``.ema`` file. The new type contains a package declaration and possibly multiple import statements for other components/component packages. The component definition then starts with the keyword ``component`` followed by the components name. The following shows the example of the ``And`` component, which represents a logical And block.  
```
package fas.basicLibrary; // package declaration

component And { // component header
    // component body
}
```
* How to add ports to a component?  

EMA components use typed ports for input and output of data. To add a port to a component we add a ``port`` block to the component's body.
```
package fas.basicLibrary;

component And {
    port
	   in Boolean in1, // first inport of type Boolean
	   in Boolean in2, // second inport of type Boolean
	   out Boolean out1; // output port of type Boolean
}
```
Each port definition consists of a ``in``/``out`` keyword, stating whether the port is an input or output port. This is followed by the type of the port, in this case all ports are of type Boolean. Each port is then given a name valid whithin the scope of the component.

* How to instantiate a component?  

In addition to ports, component types can contain instances of other components. To instantiate a component we first have to import it's type. We can then use the keyword ``instance`` to create an instance of this type and name it. To distinguish instances from their corresponding types we use lower case letters to start the names of instances. The following shows a component which uses two instances of the previously defined ``And`` component. These will become subcomponents of the enclosing component.
```
package fas;
import fas.basicLibrary.And; // import the And component

component Demo {
    port
        in Boolean in1,
        in Boolean in2,
        in Boolean in3,
        out Boolean out1;

    instance And and1; // create first And instance
    instance And and2; // create second And instance
}
```

* How to connect components?  

Components are connected by connecting their ports. For this we can reference the ports of a subcomponent by their qualified names. We then connect them by using the ``connect`` keyword. Data will then be passed along these connections.
```
package fas;
import fas.basicLibrary.And;

component Demo {
    port
        in Boolean in1,
        in Boolean in2,
        in Boolean in3,
        out Boolean out1;

    instance And and1;
    instance And and2;

    /* Connect first input port to the input of the first subcomponent */
    connect in1 -> and1.in1;
    connect in2 -> and1.in2;

    /* Connect output of the first subcomponent to the second subcomponent */
    connect and1.out1 -> and2.in1;
    connect in2 -> and2.in2;

    /* Connect output of the second subcomponent to the output port of this component */
    connect and2.out1 -> out1;
}
```
* How to deal with component and port arrays?  

To support reuse component types can contain arrays of ports. To create a port array we allow the user to pass a parameter to the component when instantiating it.
```
package fas.basicLibrary;

component And<N1 n=2> { // pass a parameter for the number of ports
    port
	   in Boolean in[n], // create n ports of type Boolean
	   out Boolean out1;
}
```
In the above example, the And component received a parameter ``n`` of type ``N1`` (a single integer number) which is used to create an array of input ports. If upon instantiation no parameter is found n defaults to 2, as seen in the example. The EMA syntax allows Java-like array syntax. The ports can be referenced individually (e.g. ``And.in[1]``) or together (using the MATLAB-like ``And.in[:]``).
Note: ``n`` starts at ``1`` NOT at ``0``. 

* How to deal with generics and default generic values?  

To make components useful in multiple contexts they can use generics. These are used to specify the types of input and output ports only upon creating the component's instance.
```
package fas.basicLibrary;

component PlusMinus<T> { // define generic parameter T
	port
	   in T in1,  // all ports will be of type T
	   in T in2,
	   out T out1;
}
```
In the above example the component ``PlusMinus`` is defined. It is used to compute an arithmetic operation on the data passed to the typed ports ``in1`` and ``in2``. The type of these ports is defined when creating an instance of the component. As an example we create an instance that computes the operation for distance values between 0m and 200m.
```
instance PlusMinus<(0 m : 200 m)> plusMinus;
```
This defines the type of all ports within the component as a range from 0 to 200, measured in meters.

* How to deal with configuration parameters?  

Component instances can be configured with initialization parameters. This can be useful to (among other things) define constants, set delay timers or fill lookup tables. Parameters are defined in rounded brackets at the end of the component head:
```
package fas.basicLibrary;

component Constant<T>(T value) {
	port
		out T out1;
}
```
The following line initializes a component with an output port of type ``Q(0:100)``:
```
instance Constant<Q(0:100)>(5) q_5;
```

Math (Sascha)
----

The Math Language is a general language which was developed to provide support for mathematical expressions and operations
for usage in the MontiCar Language Familiy. However, it is developed as a standalone language which can therefore also be used in other projects which utilize MontiCore. In the following basic features of the Math Language are explained in combination with code examples to show how these features can be used. If you need further information related to the integration of the Math Language into another language you can consult the MontiCore documentary, or examine the EmbeddedMontiArcMath project, which extends the EmbeddedMontiArc Language with behaviour by using the Math Language.  

* Basic structue of valid Math Language files  
```
package Generation; // package declaration
// other scripts can be imported by using "import packagename.scriptname;"
script MathExpressions //script name
 // instructions go here

end // end of script
```

* How to create a matrix?  
 Matrices can be written in two different ways:  
 They can either be written like the following where each row element is separated by using a `,` and each column is separated by using  a `;`:
```
package Generation; // package declaration
script MathExpressions //script name
 Q^{2,2} = [1,0; 0,1];// identity matrix

end // end of script
```
 or each row element can be separated by using ` ` which is a space character:
```
package Generation; // package declaration
script MathExpressions //script name
 Q^{2,2} = [1 0; 0 1];// identity matrix

end // end of script
```

* What Types of matrices are possible?  
  Sparse matrices like triangular and diagonal matrix.  
  TODO add examples for this  


* How to select values from a matrix?
```
package Generation; // package declaration
script MathExpressions //script name
 Q^{2,2} mat = [1,0; 0,1];// identity matrix
 Q value = mat(1,1); //get first element from matrix
end // end of script
```
* Listing of all supported opertions (+,*, ^, \, .*, .\, ...)  
 Addition:  
```
package Generation; // package declaration
script MathExpressions //script name
 Q^{2,2} mat1 = [1,0; 0,1];// identity matrix
 Q^{2,2} mat2 = [2,2; 2,2];
 Q^[2,2] matRes= mat1 + mat2; // add both matrices
end // end of script
```
 Multiplication:
```
package Generation; // package declaration
script MathExpressions //script name
 Q^{2,2} mat1 = [1,0; 0,1];// identity matrix
 Q^{2,2} mat2 = [2,2; 2,2];
 Q^[2,2] matRes= mat1 * mat2; // add both matrices
end // end of script
```
 Power of:  
```
package Generation; // package declaration
script MathExpressions //script name
 Q^{2,2} mat1 = [1,0; 0,1];// identity matrix
 Q^[2,2] matRes= mat1.^ 2; // does power of 2 for each element inside of the matrix
end // end of script
```
 Equation Solving:
```
package Generation; // package declaration
script MathExpressions //script name
 Q^[3,1] matRes = [3 6 2; 1 2 8; 7 9 4] \  [2;3;4]; // Solves Ax=b for x
end // end of script
```

 Element-Wise Multiplication:  
```
package Generation; // package declaration
script MathExpressions //script name
 Q^[3,3] matRes = [3 6 2; 1 2 8; 7 9 4] .*; // multiplies
end // end of script
```
 Element-Wise Division:

 Element-Wise Power of:


* Listing of all supported Octave functions (eig, diag, ...)


* How to deal with units? (compatiblity, conversion of units)  

Currently all units will be automatically converted to the SI base units  
See https://en.wikipedia.org/wiki/SI_base_unit for reference  
An example of numbers with different units that can be added together is presented in the following:  
```
Q(Okm : 5km) bigDistance = 4 km;
Q(0m : 5m) smallDistance = 4 m;
Q(0m : 10000m) sumDistance = bigDistance + smallDistance;
```
  
This results in `sumDistance` containing `4004m`. When changing the example to:
```
Q(Om : 5km) bigDistance = 4 km;  
Q(0m : 5m) smallDistance = 4000 m;  
Q(0mm : 10km) sumDistance = bigDistance + smallDistance;
```

**If a datatype has a unit, e.g. Q(0km : 5km), than also the assigned value must have a unit!**

Note: The compiler removes the units to support blas operations, but in the model all units must be present. The reason is to have better readability such as `distance < 40 mm` is better understandable as `distance < 40` (in Java and C we never know what unit a value represents).

`sumDistance` still contains `4004m` as everything is converted to base units internally, which avoids manually converting  
units like km to m. Notice that in both cases `smallDistance` stores `4000m`.

> Also show an example with `Z`, `N`, `R` and also one where you define the resolution `min:res:max`

> Example for `for` and `while`-loops and `if`-condition are missing.

https://de.mathworks.com/help/matlab/ref/if.html

https://de.mathworks.com/help/matlab/ref/for.html

https://de.mathworks.com/help/matlab/ref/while.html

internal unit conversions.  

EmbeddedMontiArcMath (Sascha)
----
* How to embedded Math language into EmbeddedMontiArc?

In the EmbeddedMontiArc section several examples of how valid models look like were ilustrated.  
These models can be extended with capabilities from the Math Language by adding an "implementation Math" section.  
Consider the following EmbeddedMontiArc model:  
```
component Delay {  
  ports in  (0:1) in1,  
        out (0:1) out1;            
}  
```

To use the Math Language an `implementation Math` section has to be added:
```
component Delay {  
  ports in  (0:1) in1,  
        out (0:1) out1;  

  implementation Math {  
    //Math code   
  }  
}  
```

The delay components dataflow is modeled by the `in1` and `out1` ports, which take a value between `0` and `1`.  
The intended functionality of the Delay component is to delay input for 1 tick, as can be guessed from the name.
To achieve, this behaviour, the component can be enriched with that functionality by using the Math Language.  
This results in the following component:  
```
component Delay {  
  ports in  (0:1) in1,  
        out (0:1) out1;  

  implementation Math {  
    static Q(0:1) delayValue=0; // default value on start  
    out1=delayValue; //set output to value of last tick  
    delayValue=in1; // store current tick value for next tick  
  }  
}  
```
When looking at this example, inside of the implementation Math section, a static variable `delayValue` which has a value between  
`0` and `1` is declared and initiliazed to `0`. Then the `out1` port of the EmbeddedMontiArc component is set to take the value  
of `delayValue`. Finally, `delayValue` is set to the current value of the `in1` port.  
Therefore, the Math Language can be used to add behaviour to a component which results in the ability of the  
EmbeddedMontiArcMath Language to defined components and their behaviour, and the ability to generate code out of these.  



To facilitate usage of ports which are definied in the EmbeddedMontiArc language, these ports are automatically adapted to be   
visible to the Math Language. If you are interested in how the language adaption exactly works you have to consults the  
MontiCore Language Workbench documentation.  
* How ports of EmbeddedMontiArc are adapted to matrices into Math language?  

Examples (Ievgen)
--------
* How to create a good component atomic component?
    * we do not use `And` or `Or` or `Not` as atomic components, since we can use this directly in the Math implementation
    * in the Math part: 
          * max. 3 operations per line (`y=!(u1 || (u2 && u3))`)
          * max. 5 lines in the Math part

EmbeddedMontiView (Fabian)
----
* What is the difference between EmbeddedMontiArc and EmbeddedMontiView?

EmbeddedMontiView is a C&C modelling language with many similarities to EmbeddedMontiArc. It is intended to be used to create views that describe structural properties of EmbeddedMontiArc models. The [view verification](https://github.com/EmbeddedMontiArc/ViewVerification) was developed to verify the conformance of views by models.

The syntax is nearly equal to EmbeddedMontiArc and allows several generalizations (for example, omitting a type by replacing it by a question mark `?`) as the following sections show.

----
* How to model abstract ports (with no name, with no data type and both)?

Here is an example view that shows these cases:
```
package example;

view ExamplePorts {
    component A {
        ports
            in Boolean port1,      //fully specified port
            in ? port2,            //port without specified Type
            out Boolean ?,         //port without specified Name
            out ? ?;               //port without both
    }
}
```

----
The following points are explained one the basis of the following model:
```
! NOTE THAT A SINGLE LETTER IS NOT A VALID NAME FOR A COMPONENT !
! This is used here to shorten the examples.
! Concatenating a 'comp' to every name is enough to make each of the following examples valid.
! Example: instance Bcomp bcomp;
```
```
package example;
import example.B;
import example.C;
import example.D;
import example.E;

component A {
    ports
        in Boolean in1,
        out Boolean out1,
        out Boolean out2;

    instance B b;
    instance D d;
    instance C e;

    connect in1 -> b.in1;
    connect b.out1 -> d.in1;
    connect b.out2 -> e.in1;
    connect d.out1 -> out1;
    connect e.out1 -> out2;
}
```
```
package example;
import example.C;

component B {
    ports
        in Boolean in1,
        out Boolean out1,
        out Boolean out2;

    instance C c;
    connect in1 -> c.in1;
    connect c.out1 -> out1, out2;
}
```
```
package example;

component C {
    ports
        in Boolean in1,
        out Boolean out1;
    
    connect in1 -> out1;
}
```
```
package example;

component D {
    ports
        in Boolean in1,
        out Boolean out1;
}
```
---
* What are abstract connectors?

While connectors in EMA represent a specific connection of one port to another, abstract connectors in EMV represent a directed chain of connectors (describing the transportation of information).
Example:
```
package example;

view ExampleConnector {
    component A {
        port in ? in1;

        component D {
            port in ? in1;
        }
        instance D d;

        connect in1 -> d.in1;
    }
}
```
The following path is matched:
A.in1 -> b.in1 -> c.in1 -> c.out1 -> b.out1 -> d.in1

---
* What are abstract effectors?

Unlike abstract connectors, abstract effectors also include computations in components, which are represented as effectors in the model.
Atomic components (components without subcomponents) automatically have effectors from every input to every output port.
Example:
```
view ExampleEffector {
    component A {
        ports
            in ? in1,
            out ? out1;

        effect in1 -> out1;
    }
}
```
The following path is matched:
A.in1 -> b.in1 -> c.in1 -> c.out1 -> b.out1 -> d.in1 -> d.out1 -> A.out1

---
* What are abstract hierarchies?

The hierarchy of components within a view does not necessarily represent the original hierarchy in the model. The usage of abstract connectors and effectors permits this abstraction.
Example:
```
view ExampleHierarchy {
    component A {
        port in ? in1;
        
        component C {
            ports
                in ? in1,
                out ? out1;
        }
        instance C c;
        
        component D {
            port in ? in1;
        }
        instance D d;
        
        connect in1 -> c.in1;
        connect c.out1 -> d.in1;
    }
}
```
The component c is a direct subcomponent of A in this view while this is not true in the original model. Furthermore, A, c and d are directly connected while c is encapsuled by b in the model.

---
* What is a textual anonymous port ($port1)?

Textual anonymous ports are used to distinguish and reference different ports whose names are unknown.
In the following example b.$port1 allows to specify the type of the connection between b and d while keeping the port anonymous.
```
package example;

view ExampleTAP {
    component B {
        ports
            out (0:1) $port1;
    }
    instance B b;

    component D { }
    instance D d;
    
    connect b.$port1 -> d;
}
```

RewriteConf
----

This is a DSL which allows writing configuration files, which can be imported by OCL files.
They bundle several preprocessing functions for OCL files which import them.  

A Conf file begins with specifying the package name. Imports to java classes for static methods
or other resources can be added. These are added in a preprocessing step to the OCL file.

Rewrite rules work as string-replacements similar to C defines. These are applied to the OCL File.   

Finally with addCoco one can specify cocos which are checked on the OCL AST.

Below is an example conf file CnCExt.conf and an ocl file whoch imports the former. 

> Modify the examples in such a way that they work for EmbeddedMontiArc and not for MontiArc anymore (since this is the EmbeddedMontiArc repo and docu)

```
package tagDef;

import de.monticore.lang.ocl.nfp.NFPHelper.*;
import de.monticore.lang.montiarc.montiarc._symboltable.*;
import java.lang.*;
import java.util.*;

rewrite " Cmp " -> " ExpandedComponentInstanceSymbol ";
rewrite " CTDef " -> " ComponentSymbol ";
rewrite " Con " -> " ConnectorSymbol ";

addCoco checkVariableNamingConnvention;
```
```
import tagDef.CnCExt;

ocl myOCLFile {
  context Integer a inv:
    ...
}
```

OCL (Ferdinand)
----
* How to begin

The constraints are saved in an .ocl file.
A most basic example of the file `myOCLFile.ocl` follows. 
The filename has to be the same as the name after the keyword `ocl`.  
**(optional)** It is possible to declare the package name and import libraries for static methods.

```
package de.mypackage;

import de.myLib;

ocl myOCLFile {
    ...
}
```
* How to define an invariant?

Invariants or Constraints belong within the ocl encapsulation.
The are declared with the keyword `inv`, an **(optional)** invariant name and `:`.
The invariant follows. Multiple Invariants are possible,
however not supported yet by the generation tool `ocl2java`.
```
ocl myOCLFile {
  inv myInvName:
        2 == 1 + 1;
}
```
* What does context mean?

It is possible to declare a context on which the invariant is checked against.
In the following example we have a variable named a of type int as context.
The invariant following will be checked against every possible instance of a.
```
ocl myOCLFile {
  context int a inv myInvName:
        a*a >= 0;
}
```
Multiple context variables and complex types are possible.
```
ocl myOCLFile {
  context Connector conn, Port p1, p2 inv myInvName:
        ... ;
}
```

* How to deal with Lists, Sets?

Below are a few examples of different possible comprehension types, the same applies to Sets.
For more possibilities look into UMLs OCL.
```
ocl myOCLFile {
  inv:
    List{-3..3} == List{-3, -2, -1, 0, 1, 2, 3};
    List{1..1} == List{1};
    List{9..5} == List{};
    List{'a'..'c'} == List{'a', 'b', 'c'};
    List{3, 5..7, 2} == List{3, 5, 6, 7, 2};
    List{3..5, 7..9} == List{3, 4, 5, 7, 8, 9};
    List{3 .. (2 + 5)} == List{3, 4, 5, 6, 7};
    
    List{x * x | x in List{1..5}} == List{1, 4, 9, 16, 25};
    List{x * x | x in List{1..8}, !even(x)} == List{1, 9, 25, 49};
}
```
* How to use quantificators?

`forall` and `exists` quantifiers are usable. However for the `ocl2java` tool the type of the quantified variable has to be set.
```
ocl myOCLFile {
  context Component comp inv:
    forall Port p in comp.ports: 
        !(p.isOutgoing && p.isIncoming);
}
```

* How to deal with implications, and other operators?
```
ocl myOCLFile {
  context Boolean a,b inv:
    !b;
    a && b;
    a || b;
    a imples b;
}
```

* Let-In Constructs

These constructs allow the user to define variables which are used in an expression.

```
ocl myOCLFile {
  context Integer a inv:
    let
      Integer b = 0 ;
    in
      a > b;
}
```



* How to describe Extra-Functional property semantics in OCL/P?

Struct DSL (Alex)
----
* What is it?

The [Struct](https://github.com/EmbeddedMontiArc/Struct) is a DSL (Domain Specific Language) for describing C-like structures:
```
package geometry;

struct Position {
  Q (-oo m : oo m) x;
  Q (-oo m : oo m) y;
}
```
A structure is basically a group of related pieces of data combined together into a single entity (struct). Pretty much like in C programming language.

* Why would I use it?

Because it makes sence to group related pieces of data into a single struct. Consider the following component:
```
package geometry;

component CenterOfSegment {
  port
    in  Q (-oo m : oo m) p1x,
    in  Q (-oo m : oo m) p1y,
    in  Q (-oo m : oo m) p2x,
    in  Q (-oo m : oo m) p2y,
    out Q (-oo m : oo m) cx,
    out Q (-oo m : oo m) cy;

  implementation Math {
    cx = 0.5 * (p1x + p2x);
    cy = 0.5 * (p1y + p2y);
  }
}
```
Since `p1x` and `p1y` represent a point of a segment it would be nice to combine them into a single entity (`Position`) and then use a single component's port to refer to this point. The same applies to `p2x` and `p2y`:
```
package geometry;

component CenterOfSegment {
  port
    in  Position p1,
    in  Position p2,
    out Position center;

  implementation Math {
    center.x = 0.5 * (p1.x + p2.x);
    center.y = 0.5 * (p1.y + p2.y);
  }
}
```

* How do I use it?

    * Create a file with `struct` extension and describe your data in there (e.g. `Position.struct`).
    * Import your struct in the `emam` file where your component resides (e.g. `MyComponent1.emam`):
        ```
        package modeling;

        import geometry.Position;

        component MyComponent1 {
            // component's declaration
        }
        ```
    * Use it as a port's type in your component:
        ```
        package modeling;

        import geometry.Position;

        component MyComponent1 {
            port
              in Position in1,
              out Position out1;

            implementation Math {
                out1.x = 2 * in1.x;
                out1.y = 2 * in1.y;
            }
        }
        ```

* Some limitations:
    * Generics are not supported. E.g. the following declaration is not valid:
        ```
        package example;

        struct Pair<T> {
            T first;
            T second;
        }
        ```  
    * Struct references must not form a cycle. E.g. the following struct declarations are illegal:
        ```
        package example;

        struct S1 {
            Q f1;
            S2 f2;
        }
        ```
        ```
        package example;

        struct S2 {
            Q f1;
            S1 f2;
        }
        ```
    * Nested structs are not supported. E.g. the following declaration is not valid:
        ```
        package example;

        struct S1 {
            Q f1;
            B f2;

            struct Nested {
                Q field1;
                B field2;
            }
        }
        ```
    * There must be one struct per file. E.g. the following declarations are not permitted in one file:
        ```
        package example;

        struct S1 {
            Q f1;
            B f2;
        }

        struct S2 {
            Q x;
            Q y;
        }
        ```
