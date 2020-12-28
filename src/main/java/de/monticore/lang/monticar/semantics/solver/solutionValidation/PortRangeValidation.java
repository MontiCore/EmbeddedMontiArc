/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSymbolicVariableSymbol;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import org.jscience.mathematics.number.Rational;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

public class PortRangeValidation {

    private final Collection<EMAMSymbolicVariableSymbol> variables;
    private List<PortRangeValidationViewModel> valids;
    private List<PortRangeValidationViewModel> invalids;

    public PortRangeValidation(Collection<EMAMSymbolicVariableSymbol> variables) {
        this.variables = variables;
        for (EMAMSymbolicVariableSymbol variable : variables) {

        }
    }

    public List<PortRangeValidationViewModel> getValids() {
        return valids;
    }

    public List<PortRangeValidationViewModel> getInvalids() {
        return invalids;
    }

    private Optional<Double> getMin(EMAMSymbolicVariableSymbol variable) {
        if (variable.getType() != null) {
            if (variable.getType().getType() instanceof ASTElementType) {
                if (variable.getType().getType().isPresentRange()) {
                    ASTRange range = variable.getType().getType().getRange();
                    return Optional.of(range.getStartValue().doubleValue());
                }
            }
        } else if (variable.getPort().isPresent()) {

        }
        return Optional.empty();
    }

    private Optional<Double> getMax(EMAMSymbolicVariableSymbol variable) {
        if (variable.getType() != null) {
            if (variable.getType().getType() instanceof ASTElementType) {
                if (variable.getType().getType().isPresentRange()) {
                    ASTRange range = variable.getType().getType().getRange();
                    return Optional.of(range.getEndValue().doubleValue());
                }
            }
        } else if (variable.getPort().isPresent()) {

        }
        return Optional.empty();
    }
}
