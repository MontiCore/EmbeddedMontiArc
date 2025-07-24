/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;

import java.util.Collection;

public class EMAMSpecificationSymbolSynth extends EMAMSpecificationSymbol {
    public EMAMSpecificationSymbolSynth(Collection<EMAMSymbolicVariableSymbol> variables, Collection<EMAMEquationSymbol> equations, Collection<EMAMInitialValueSymbol> initialValues, Collection<EMAMInitialGuessSymbol> initialGuesses) {
        super(variables, equations, initialValues, initialGuesses);
    }

    public EMAMSpecificationSymbolSynth(Collection<EMAMSymbolicVariableSymbol> variables, Collection<EMAMEquationSymbol> equations) {
        super(variables, equations);
    }

    @Override
    protected void resolveComponentInstance() {
        resolvedComponentInstance = true;
    }
}
