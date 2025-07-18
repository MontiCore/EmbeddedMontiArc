/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSymbolicVariableSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class SolutionValidation {

    public static boolean isValid(Collection<EMAMSymbolicVariableSymbol> variables,
                                  Collection<EMAMSymbolicVariableSymbol> inports,
                                  Map<EMAMSymbolicVariableSymbol, MathExpressionSymbol> solution) {
        PortRangeValidation portRangeValidation = new PortRangeValidation(variables, inports);

        List<PortRangeValidationViewModel> invalids = portRangeValidation.getInvalids();

        invalids.toString();

        return true;
    }

}
