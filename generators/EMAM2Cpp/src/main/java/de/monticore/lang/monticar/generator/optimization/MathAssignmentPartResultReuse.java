/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.math._symboltable.MathAssignmentOperator;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverterMethodGeneration;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 */
public class MathAssignmentPartResultReuse implements MathOptimizationRule {
    MathStatementsSymbol currentMathStatementsSymbol = null;
    List<MathExpressionSymbol> encounteredSymbolInstances = new ArrayList<>();
    Map<MathExpressionSymbol, String> symbolMap = new HashMap();
    public static int currentId = 0;
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
            Log.debug(mathExpressionSymbol.getClass().getName() + " " + mathExpressionSymbol.getTextualRepresentation(), "optimize Symbol not handled");
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
            Log.debug(mathExpressionSymbol.getClass().getName() + " " + mathExpressionSymbol.getTextualRepresentation(), "Symbol not handled");
        }
    }

    public void optimize(MathMatrixNameExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (encounteredSymbolInstances.contains(mathExpressionSymbol)) {
            Log.debug("Found Same Symbol", "MathAssignmentPartResultReuses");
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
            Log.debug("Added " + mathExpressionSymbol.getTextualRepresentation() + " to encounterSymbolInstances", "MathAssignmentPartResultReuse");
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
