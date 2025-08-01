/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSymbolicVariableSymbol;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.types2._ast.ASTElementType;

import java.util.*;
import java.util.stream.Collectors;

public class PortRangeValidation {

    private PortRangeValidationViewModel valid;
    private List<PortRangeValidationViewModel> invalids = new ArrayList<>();

    public PortRangeValidation(Collection<EMAMSymbolicVariableSymbol> variables,
                               Collection<EMAMSymbolicVariableSymbol> inports) {
        Map<EMAMSymbolicVariableSymbol, Double> portRangeMin = new HashMap<>();
        Map<EMAMSymbolicVariableSymbol, Double> portRangeMax = new HashMap<>();

        Map<EMAMSymbolicVariableSymbol, PortLowerRangeViewModel> validLowerRanges = new HashMap<>();
        Map<EMAMSymbolicVariableSymbol, PortUpperRangeViewModel> validUpperRanges = new HashMap<>();

        for (EMAMSymbolicVariableSymbol variable : variables) {
            Optional<Double> min = getMin(variable);
            Optional<Double> max = getMax(variable);
            if (min.isPresent()) {
                portRangeMin.put(variable, min.get());
                validLowerRanges.put(variable,
                        new PortLowerRangeViewModel(variable.getTextualRepresentation(), min.get().toString()));
            }
            if (max.isPresent()) {
                portRangeMax.put(variable, max.get());
                validUpperRanges.put(variable,
                        new PortUpperRangeViewModel(variable.getTextualRepresentation(), max.get().toString()));
            }
        }
        this.valid = new PortRangeValidationViewModel(validLowerRanges.values(), validUpperRanges.values());

        for (EMAMSymbolicVariableSymbol variable : variables) {
            PortLowerRangeViewModel lower = validLowerRanges.get(variable);
            PortUpperRangeViewModel upper = validUpperRanges.get(variable);
            if (lower == null && upper == null) continue;

            Set<PortLowerRangeViewModel> otherLowers = validLowerRanges.entrySet().stream()
                    .filter(e -> e.getKey() != variable)
                    .map(Map.Entry::getValue)
                    .collect(Collectors.toSet());
            Set<PortUpperRangeViewModel> otherUppers = validUpperRanges.entrySet().stream()
                    .filter(e -> e.getKey() != variable)
                    .map(Map.Entry::getValue)
                    .collect(Collectors.toSet());

            if (lower != null)
                this.invalids.add(new PortRangeValidationViewModel(otherLowers, otherUppers,
                        new PortUpperRangeViewModel(lower.getName(), lower.getLowerLimit())));

            if (upper != null)
                this.invalids.add(new PortRangeValidationViewModel(otherLowers, otherUppers,
                        new PortLowerRangeViewModel(upper.getName(), upper.getUpperLimit())));
        }
    }

    public PortRangeValidationViewModel getValid() {
        return valid;
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
            if(variable.getPort().get().getAstNode().isPresent()
                    && variable.getPort().get().getAstNode().get() instanceof ASTPort){
                ASTPort astPort = (ASTPort) variable.getPort().get().getAstNode().get();
                if (astPort.getType() instanceof ASTElementType) {
                    if (((ASTElementType) astPort.getType()).isPresentRange()) {
                        ASTRange range = ((ASTElementType) astPort.getType()).getRange();
                        return Optional.of(range.getStartValue().doubleValue());
                    }
                }
            }
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
            if(variable.getPort().get().getAstNode().isPresent()
                    && variable.getPort().get().getAstNode().get() instanceof ASTPort){
                ASTPort astPort = (ASTPort) variable.getPort().get().getAstNode().get();
                if (astPort.getType() instanceof ASTElementType) {
                    if (((ASTElementType) astPort.getType()).isPresentRange()) {
                        ASTRange range = ((ASTElementType) astPort.getType()).getRange();
                        return Optional.of(range.getEndValue().doubleValue());
                    }
                }
            }
        }
        return Optional.empty();
    }
}
