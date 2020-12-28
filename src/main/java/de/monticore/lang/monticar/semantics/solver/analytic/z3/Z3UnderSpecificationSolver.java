/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.analytic.z3;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.monticar.semantics.solver.analytic.UnderSpecificationSolver;

import java.util.Collection;
import java.util.Map;

public class Z3UnderSpecificationSolver implements UnderSpecificationSolver {

    @Override
    public Map<String, String> solve(Collection<EMAMEquationSymbol> system, Collection<String> variables) {
        return null;
    }
}
