
[![pipeline status](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/badges/master/pipeline.svg)](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/commits/master)
[![coverage report](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/badges/master/coverage.svg)](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/commits/master)

# MontiMathOpt

## Summary

MontiMathOpt extends the math description language [MontiMath](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMath) by optimization features.

MontiMathOpt allows to describe mathematical optimization problems such as minimizing or maximizing an objective function under given constraints. 

![Example Minimization Problem](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/raw/994d5a727661dbe7e1508e404bffa502afd8f54a/doc/img/OptimizationProblem.png "Example Minimization Problem")


## Syntax

The systax is defined in the grammar file [MathOpt.mc4](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/blob/master/src/main/grammars/de/monticore/lang/MathOpt.mc4)


```
Q cost = minimize(Q^{3, 2} x)
  sum(c .* x);
subject to
  sum(x, 2) == A;
  sum(x, 1) == b;
   x >= 0;
end
```

## Language Hierachy

MontiMathOpt extends MontiMath:

![MathOpt Language Hierachy](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/raw/master/doc/img/mathopt.png)

## Development Hints

This project contains 3 important entry points:
- grammar file [MathOpt.mc4](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/blob/master/src/main/grammars/de/monticore/lang/MathOpt.mc4)
- symbol table creation [MathOptSymbolTableCreator.java](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/blob/master/src/main/java/de/monticore/lang/mathopt/_symboltable/MathOptSymbolTableCreator.java)
- optimization symbol [MathOptimizationStatementSymbol.java](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/blob/master/src/main/java/de/monticore/lang/mathopt/_symboltable/MathOptimizationStatementSymbol.java)
- context conditions [MathOptCocos.java](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/blob/master/src/main/java/de/monticore/lang/mathopt/_cocos/MathOptCocos.java)

## See Also

[EMAMOpt](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/EmbeddedMontiArcMathOpt)

[EMAMOpt2Cpp](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAMOpt2Cpp)

[Master-Thesis](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMathOpt/blob/master/doc/master_thesis_richter.pdf)
