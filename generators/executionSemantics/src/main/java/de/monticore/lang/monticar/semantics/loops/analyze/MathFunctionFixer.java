/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSpecificationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.CopyEMAMMathExpressionSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolVisitor;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.semantics.setup.Delegate;
import de.monticore.symboltable.Symbol;

import java.util.*;

public class MathFunctionFixer implements EMAMMathExpressionSymbolVisitor {

    private List<String> toHandle = Arrays.asList("sum", "product");

    public static EMAMSpecificationSymbol replaceInnerArrayByIndividualPorts(EMAMSpecificationSymbol specificationSymbol) {
        Collection<EMAMEquationSymbol> newEquations = new ArrayList<>();
        MathFunctionFixer visitor = new MathFunctionFixer();
        for (EMAMEquationSymbol equation : specificationSymbol.getEquations()) {
            EMAMEquationSymbol copy = Delegate.copyMathExpressionSymbol(equation);
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
        if (!toHandle.contains(nameToAccess)) return;

        ListIterator<MathMatrixAccessSymbol> iterator = node.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().listIterator();
        while (iterator.hasNext()) {
            MathMatrixAccessSymbol next = iterator.next();
            if (next.getMathExpressionSymbol().isPresent()) {
                if (next.getMathExpressionSymbol().get() instanceof MathNameExpressionSymbol) {
                    String nameToResolveValue = ((MathNameExpressionSymbol) next.getMathExpressionSymbol().get()).getNameToResolveValue();
                    Optional<EMAPortArraySymbol> resolve =
                            node.getEnclosingScope().resolve(nameToResolveValue, EMAPortArraySymbol.KIND);
                    if (resolve.isPresent()) {
                        next.setMathExpressionSymbol(new MathNameExpressionSymbol(nameToResolveValue + "(1)"));
                        for (int i = 2; i <= resolve.get().getDimension(); i++) {
                            iterator.add(new MathMatrixAccessSymbol(
                                    new MathNameExpressionSymbol(String.format("%s(%s)", nameToResolveValue, i))));
                        }
                    }


                    // TODO handle Arrays equally
                }
            }
        }

    }

    private Set<MathExpressionSymbol> visitedSymbols = new HashSet<>();

    @Override
    public Set<MathExpressionSymbol> getVisitedSymbols() {
        return this.visitedSymbols;
    }
}
