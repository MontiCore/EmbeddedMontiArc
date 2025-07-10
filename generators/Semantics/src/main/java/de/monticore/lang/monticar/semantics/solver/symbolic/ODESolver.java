/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.symbolic;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;

import java.util.Collection;
import java.util.Map;

public interface ODESolver {

    public Map<String, String> solve(Collection<EMAMEquationSymbol> system, Collection<String> variables
            , Map<String, Double> startValues);


}
