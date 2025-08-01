/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.analytic;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;

import java.util.Collection;
import java.util.Map;

public interface PolynomSolver {

    public Map<String, String> solve(Collection<EMAMEquationSymbol> system, Collection<String> variables);

}
