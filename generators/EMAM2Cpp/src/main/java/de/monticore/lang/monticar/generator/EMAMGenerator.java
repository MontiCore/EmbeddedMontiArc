/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

public interface EMAMGenerator extends Generator<EMAComponentInstanceSymbol> {

    boolean useAlgebraicOptimizations();

    void setUseAlgebraicOptimizations(boolean useAlgebraicOptimizations);

    boolean useThreadingOptimizations();

    void setUseThreadingOptimization(boolean useThreadingOptimizations);

    MathCommandRegister getMathCommandRegister();

    void setMathCommandRegister(MathCommandRegister mathCommandRegister);
}
