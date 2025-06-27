/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.math;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialGuessSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialValueSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolVisitor;
import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;

public class NameReplacer implements EMAMMathExpressionSymbolVisitor {

    public static void replaceNames(MathExpressionSymbol expression, Function<String, String> nameMapping) {
        NameReplacer nameReplacer = new NameReplacer(nameMapping);
        nameReplacer.handle(expression);
    }

    private final Function<String, String> nameMapping;

    public NameReplacer(Function<String, String> nameMapping) {
        this.nameMapping = nameMapping;
    }

    @Override
    public void visit(MathNameExpressionSymbol node) {
        Optional<String> newName = nameOf(node.getNameToAccess());
        if (newName.isPresent())
            node.setNameToAccess(newName.get());
    }

    @Override
    public void visit(MathAssignmentExpressionSymbol node) {
        Optional<String> newName = nameOf(node.getNameOfMathValue());
        if (newName.isPresent())
            node.setNameOfMathValue(newName.get());
    }

    @Override
    public void visit(MathMatrixNameExpressionSymbol node) {
        Optional<String> newName = nameOf(node.getNameToAccess());
        if (newName.isPresent())
            node.setNameToAccess(newName.get());
    }

    @Override
    public void visit(EMAMInitialGuessSymbol node) {
        Optional<String> newName = nameOf(node.getNameToAccess());
        if (newName.isPresent())
            node.setNameToAccess(newName.get());
    }

    @Override
    public void visit(EMAMInitialValueSymbol node) {
        Optional<String> newName = nameOf(node.getNameToAccess());
        if (newName.isPresent())
            node.setNameToAccess(newName.get());
    }

    private Optional<String> nameOf(String name) {
        String newName = nameMapping.apply(name);
        if (newName == null || newName.equals(""))
            return Optional.empty();
        return Optional.of(newName);
    }

    Set<MathExpressionSymbol> visitedSymbols = new HashSet<>();

    @Override
    public Set<MathExpressionSymbol> getVisitedSymbols() {
        return visitedSymbols;
    }
}