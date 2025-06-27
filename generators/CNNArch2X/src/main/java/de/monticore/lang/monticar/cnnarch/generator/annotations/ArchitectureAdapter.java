/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator.annotations;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.IODeclarationSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.symboltable.CommonSymbol;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkNotNull;

public class ArchitectureAdapter implements NNArchitecture {

    private ArchitectureSymbol architectureSymbol;

    public ArchitectureAdapter(final ArchitectureSymbol architectureSymbol) {
        checkNotNull(architectureSymbol);
        this.architectureSymbol = architectureSymbol;
    }

    @Override
    public String getName() {
        return architectureSymbol.getName();
    }

    @Override
    public List<String> getInputs() {
        return getIOInputSymbols().stream()
            .map(CommonSymbol::getName)
            .collect(Collectors.toList());
    }

    @Override
    public List<String> getOutputs() {
        return getIOOutputSymbols().stream()
            .map(CommonSymbol::getName)
            .collect(Collectors.toList());
    }

    @Override
    public Map<String, List<Integer>> getDimensions() {
        return getAllIOSymbols().stream().collect(Collectors.toMap(CommonSymbol::getName,
            s-> ((IODeclarationSymbol) s.getDeclaration()).getType().getDimensions()));
    }

    @Override
    public Map<String, Range> getRanges() {
        return getAllIOSymbols().stream().collect(Collectors.toMap(CommonSymbol::getName,
            s -> astRangeToTrainRange(((IODeclarationSymbol) s.getDeclaration()).getType().getDomain().getRangeOpt().orElse(null))));
    }

    @Override
    public Map<String, String> getTypes() {
        return getAllIOSymbols().stream().collect(Collectors.toMap(CommonSymbol::getName,
            s -> ((IODeclarationSymbol) s.getDeclaration()).getType().getDomain().getName()));
    }

    public ArchitectureSymbol getArchitectureSymbol() {
        return this.architectureSymbol;
    }

    private Range astRangeToTrainRange(final ASTRange range) {
        if (range == null || (range.hasNoLowerLimit() && range.hasNoUpperLimit())) {
            return Range.withInfinityLimits();
        } else if (range.hasNoUpperLimit() && !range.hasNoLowerLimit()) {
            double lowerLimit = range.getStartValue().doubleValue();
            return Range.withUpperInfinityLimit(lowerLimit);
        } else if (!range.hasNoUpperLimit() && range.hasNoLowerLimit()) {
            double upperLimit = range.getEndValue().doubleValue();
            return Range.withLowerInfinityLimit(upperLimit);
        } else {
            double lowerLimit = range.getStartValue().doubleValue();
            double upperLimit = range.getEndValue().doubleValue();
            return Range.withLimits(lowerLimit, upperLimit);
        }
    }

    private List<VariableSymbol> getIOOutputSymbols() {
        return architectureSymbol.getOutputs();
    }

    private List<VariableSymbol> getIOInputSymbols() {
        return architectureSymbol.getInputs();
    }

    private List<VariableSymbol> getAllIOSymbols() {
        List<VariableSymbol> ioSymbols = new ArrayList<>();
        ioSymbols.addAll(getIOOutputSymbols());
        ioSymbols.addAll(getIOInputSymbols());
        return ioSymbols;
    }
}
