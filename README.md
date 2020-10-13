<!-- (c) https://github.com/MontiCore/monticore -->
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAMOpt2Cpp/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAMOpt2Cpp/badges/master/coverage.svg)

# EMAMOpt2Cpp

## Summary

EMAMOpt2Cpp is a code generator which converts EMAMOpt models to C++ code. It extends EMAM2Cpp which generates C++ code from EMAM models. 

It features:
- support of multiple optimization solvers
- support for different optimization problem classes
- CMake file generation for needed libraries 

## Class Hierachy

![Overview Classes](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAMOpt2Cpp/raw/master/doc/emamopt2cpp.png)

## Development Hints


### Add a new solver

Adding a new solver generator involves 2 steps:

**Step 1.)**

Implement the [SolverGeneratorImplementation.java](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAMOpt2Cpp/blob/master/src/main/java/de/monticore/lang/monticar/generator/cpp/optimizationSolver/solver/SolverGeneratorImplementation.java) interface for the corresponding problem classes in your new created class.

 **Step 2.)**
 
 Add your solver to the solver generator factory [SolverGeneratorFactory.java](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAMOpt2Cpp/blob/master/src/main/java/de/monticore/lang/monticar/generator/cpp/optimizationSolver/solver/factory/SolverGeneratorFactory.java).
 
 **Remarks**
 
 Do not forget to create unit tests ;)
 
 ### Combining Armadillo and Your Solver Code
 The tricky part was to combine the data types from Armadillo with for example IPOpt. 
 Unfortunatly it is not possible to extend Armadillos data type _mat_ for matrices easily. 
 
 For IPopt and CPLEX Armadillo's generic data type _field_ was used. The generic type was solver specific. In the case of IPOpt it was _admat_ as type for the optimization variable. 
 
 The problem with _field_ is that not all matrix operations are implemented. Thus they have to be implemented manually. In the case some functions are missing they can be implemented here: [ADMat.h](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAMOpt2Cpp/blob/master/src/main/resources/template/optimizationSolver/ipopt/ADMat.h)


## See Also

[MathOpt](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt)

[EMAMOpt](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/EmbeddedMontiArcMathOpt)

[Master-Thesis](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/blob/master/doc/master_thesis_richter.pdf)
