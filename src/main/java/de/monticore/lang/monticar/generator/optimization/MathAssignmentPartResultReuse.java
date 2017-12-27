package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.math.math._symboltable.MathAssignmentOperator;
import de.monticore.lang.math.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverterMethodGeneration;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * @author Sascha Schneiders
 */
public class MathAssignmentPartResultReuse implements MathOptimizationRule {
    MathStatementsSymbol currentMathStatementsSymbol = null;
    List<MathExpressionSymbol> encounteredSymbolInstances = new ArrayList<>();
    Map<MathExpressionSymbol, String> symbolMap = new HashMap();
    int currentId = 0;
    MathExpressionSymbol startMathExpressionSymbol = null;

    @Override
    public void optimize(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (mathExpressionSymbol == null) {

        } else if (mathExpressionSymbol.isAssignmentExpression()) {
            optimize((MathAssignmentExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isArithmeticExpression()) {
            optimize((MathArithmeticExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isMatrixExpression()) {
            optimize((MathMatrixExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else {
            System.out.println("Symbol not handled: " + mathExpressionSymbol.getClass().getName() + " " + mathExpressionSymbol.getTextualRepresentation());
        }
    }

    @Override
    public void optimize(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions, MathStatementsSymbol mathStatementsSymbol) {
        currentMathStatementsSymbol = mathStatementsSymbol;
        encounteredSymbolInstances.clear();
        symbolMap.clear();
        startMathExpressionSymbol = mathExpressionSymbol;
        optimize(mathExpressionSymbol, precedingExpressions);
    }

    public void optimize(MathAssignmentExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimize(mathExpressionSymbol.getExpressionSymbol(), precedingExpressions);
    }

    public void optimize(MathArithmeticExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimize(mathExpressionSymbol.getLeftExpression(), precedingExpressions);
        optimize(mathExpressionSymbol.getRightExpression(), precedingExpressions);
    }

    public void optimize(MathMatrixExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (mathExpressionSymbol.isMatrixNameExpression()) {
            optimize((MathMatrixNameExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else {
            System.out.println("Symbol not handled: " + mathExpressionSymbol.getClass().getName() + " " + mathExpressionSymbol.getTextualRepresentation());
        }
    }

    public void optimize(MathMatrixNameExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (encounteredSymbolInstances.contains(mathExpressionSymbol)) {
            System.out.println("Found Same Symbol");
            String name = "";
            if (!symbolMap.containsKey(mathExpressionSymbol)) {
                symbolMap.put(mathExpressionSymbol, name = getReplacementName(currentId++));
            } else {
                name = symbolMap.get(mathExpressionSymbol);
            }

            currentMathStatementsSymbol.replaceMathExpression(constructMathExpressionSymbolForName(name), mathExpressionSymbol);
            currentMathStatementsSymbol.addMathExpressionBefore(constructMathExpressionSymbolReplacement(name, mathExpressionSymbol), startMathExpressionSymbol);
            --ComponentConverterMethodGeneration.currentGenerationIndex;
        } else {
            encounteredSymbolInstances.add(mathExpressionSymbol);
            System.out.println("Added " + mathExpressionSymbol.getTextualRepresentation() + " to encounterSymbolInstances");
            optimize(mathExpressionSymbol.getMathMatrixAccessOperatorSymbol(), precedingExpressions);
        }
    }

    public static String getReplacementName(int currentId) {
        return "_I_" + currentId;
    }

    public static MathExpressionSymbol constructMathExpressionSymbolForName(String name) {
        return new MathNameExpressionSymbol(name);
    }

    public static MathExpressionSymbol constructMathExpressionSymbolReplacement(String name, MathExpressionSymbol mathExpressionSymbol) {
        MathAssignmentExpressionSymbol mathAssignmentExpressionSymbol = new MathAssignmentExpressionSymbol();
        mathAssignmentExpressionSymbol.setNameOfMathValue("auto " + name);//Use auto for C++ automatic type deduction
        mathAssignmentExpressionSymbol.setAssignmentOperator(new MathAssignmentOperator("="));
        mathAssignmentExpressionSymbol.setExpressionSymbol(mathExpressionSymbol);
        return mathAssignmentExpressionSymbol;
    }
}
