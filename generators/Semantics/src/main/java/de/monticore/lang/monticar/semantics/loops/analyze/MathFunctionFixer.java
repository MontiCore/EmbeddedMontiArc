/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSpecificationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.CopyEMAMMathExpressionSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolVisitor;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public class MathFunctionFixer implements EMAMMathExpressionSymbolVisitor {

    public static EMAMSpecificationSymbol fixFunctions(EMAMSpecificationSymbol specificationSymbol) {
        Collection<EMAMEquationSymbol> newEquations = new ArrayList<>();
        MathFunctionFixer visitor = new MathFunctionFixer();
        for (EMAMEquationSymbol equation : specificationSymbol.getEquations()) {
            EMAMEquationSymbol copy = CopyEMAMMathExpressionSymbol.copy(equation);
            visitor.handle(copy);
            newEquations.add(copy);
        }
        return new EMAMSpecificationSymbol(
                specificationSymbol.getVariables(),
                newEquations,
                specificationSymbol.getInitialValues(),
                specificationSymbol.getInitialGuesses()
        );
    }

    @Override
    public void visit(MathMatrixNameExpressionSymbol node) {
        String nameToAccess = node.getNameToAccess();
        switch (nameToAccess) {
            case "product":
                nameToAccess.toString();
            case "sum":
                nameToAccess.toString();
        }
    }

    private Set<MathExpressionSymbol> visitedSymbols = new HashSet<>();

    @Override
    public Set<MathExpressionSymbol> getVisitedSymbols() {
        return this.visitedSymbols;
    }
}
