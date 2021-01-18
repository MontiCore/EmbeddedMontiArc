/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolReplacementVisitor;
import de.monticore.lang.mathopt._symboltable.visitor.MathOptExpressionSymbolReplacementVisitor;

import java.util.*;
import java.util.function.Function;

public class EMAMMathExpressionSymbolReplacementVisitor extends MathOptExpressionSymbolReplacementVisitor
        implements EMAMMathExpressionSymbolVisitor {

    public static void replace(MathExpressionSymbol mathExpressionSymbol,
                               Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        EMAMMathExpressionSymbolReplacementVisitor replacementVisitor =
                new EMAMMathExpressionSymbolReplacementVisitor(replacementMap);
        replacementVisitor.handle(mathExpressionSymbol);
    }

    public static void replace(MathExpressionSymbol mathExpressionSymbol,
                               Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        EMAMMathExpressionSymbolReplacementVisitor replacementVisitor =
                new EMAMMathExpressionSymbolReplacementVisitor(replacementFunction);
        replacementVisitor.handle(mathExpressionSymbol);
    }

    public static void replace(MathStatementsSymbol mathStatementsSymbol,
                               Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        EMAMMathExpressionSymbolReplacementVisitor replacementVisitor =
                new EMAMMathExpressionSymbolReplacementVisitor(replacementMap);
        replacementVisitor.handle(mathStatementsSymbol);
    }

    public static void replace(MathStatementsSymbol mathStatementsSymbol,
                               Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        EMAMMathExpressionSymbolReplacementVisitor replacementVisitor =
                new EMAMMathExpressionSymbolReplacementVisitor(replacementFunction);
        replacementVisitor.handle(mathStatementsSymbol);
    }

    public EMAMMathExpressionSymbolReplacementVisitor(Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        super(replacementMap);
    }

    public EMAMMathExpressionSymbolReplacementVisitor(Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        super(replacementFunction);
    }

    @Override
    public void visit(EMAMSpecificationSymbol node) {
        Collection<EMAMSymbolicVariableSymbol> variables = new ArrayList<>();
        Collection<EMAMInitialValueSymbol> initialValues = new ArrayList<>();
        Collection<EMAMInitialGuessSymbol> initialGuesses = new ArrayList<>();
        Collection<EMAMEquationSymbol> equations = new ArrayList<>();

        for (EMAMSymbolicVariableSymbol variable : node.getVariables())
            variables.add(get(variable));
        for (EMAMInitialValueSymbol initialValue : node.getInitialValues())
            initialValues.add(get(initialValue));
        for (EMAMInitialGuessSymbol initialGuess : node.getInitialGuesses())
            initialGuesses.add(get(initialGuess));
        for (EMAMEquationSymbol equation : node.getEquations())
            equations.add(get(equation));

        node.setVariables(variables);
        node.setInitialValues(initialValues);
        node.setInitialGuesses(initialGuesses);
        node.setEquations(equations);
    }

    @Override
    public void visit(EMAMSymbolicVariableSymbol node) {
        if (node.getType() != null) node.setType(get(node.getType()));
    }

    @Override
    public void visit(EMAMEquationSymbol node) {
        if (node.getLeftExpression() != null) node.setLeftExpression(get(node.getLeftExpression()));
        if (node.getRightExpression() != null) node.setRightExpression(get(node.getRightExpression()));
    }

    @Override
    public void visit(EMAMInitialGuessSymbol node) {
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            node.setMathMatrixAccessOperatorSymbol(get(node.getMathMatrixAccessOperatorSymbol()));
        if (node.getValue() != null) node.setValue(get(node.getValue()));
    }

    @Override
    public void visit(EMAMInitialValueSymbol node) {
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            node.setMathMatrixAccessOperatorSymbol(get(node.getMathMatrixAccessOperatorSymbol()));
        if (node.getValue() != null) node.setValue(get(node.getValue()));
    }

    @Override
    public void visit(MathStringExpression node) {
        List<MathMatrixAccessSymbol> previousExpressionSymbols = new ArrayList<>();
        for (MathMatrixAccessSymbol previousExpressionSymbol : node.getPreviousExpressionSymbols())
            previousExpressionSymbols.add(get(previousExpressionSymbol));
        node.setPreviousExpressionSymbols(previousExpressionSymbols);
    }

    @Override
    public void visit(MathChainedExpression node) {
        if (node.getFirstExpressionSymbol() != null)
            node.setFirstExpressionSymbol(get(node.getFirstExpressionSymbol()));
        if (node.getSecondExpressionSymbol() != null)
            node.setSecondExpressionSymbol(get(node.getSecondExpressionSymbol()));
    }
}
