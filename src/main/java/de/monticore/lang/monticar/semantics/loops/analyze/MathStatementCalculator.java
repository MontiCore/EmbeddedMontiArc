/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._ast.ASTMathMatrixNameExpression;
import de.monticore.lang.math._symboltable.MathAssignmentOperator;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.copy.CopyMathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.matrix._ast.ASTMathMatrixAccessExpression;
import de.monticore.lang.monticar.resolution._ast.ASTResolution;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.se_rwth.commons.logging.Log;

import java.util.ListIterator;
import java.util.Optional;

public class MathStatementCalculator {

    // TODO

    private static final String packageName = "de.monticore.lang.monticar.semantics.basicLibrary.";
    private static final String And = packageName + "And";
    private static final String Constant = packageName + "Constant";
    private static final String Delay = packageName + "Delay";
    private static final String Derivation = packageName + "Derivation";
    private static final String Division = packageName + "Division";
    private static final String Equals = packageName + "Equals";
    private static final String Gain = packageName + "Gain";
    private static final String Greater = packageName + "Greater";
    private static final String GreaterEquals = packageName + "GreaterEquals";
    private static final String Integration = packageName + "Integration";
    private static final String Less = packageName + "Less";
    private static final String LookUp = packageName + "LookUp";
    private static final String Max = packageName + "Max";
    private static final String Memory = packageName + "Memory";
    private static final String Min = packageName + "Min";
    private static final String MinusPlus = packageName + "MinusPlus";
    private static final String Mod = packageName + "Mod";
    private static final String MultDiv = packageName + "MultDiv";
    private static final String Multiplication = packageName + "Multiplication";
    private static final String Not = packageName + "Not";
    private static final String Or = packageName + "Or";
    private static final String PlusMinus = packageName + "PlusMinus";
    private static final String PlusMinusPlus = packageName + "PlusMinusPlus";
    private static final String Saturation = packageName + "Saturation";
    private static final String Smaller = packageName + "Smaller";
    private static final String SmallerEquals = packageName + "SmallerEquals";
    private static final String Sum = packageName + "Sum";
    private static final String Switch = packageName + "Switch";
    private static final String SwitchB = packageName + "SwitchB";
    private static final String SwitchM = packageName + "SwitchM";
    private static final String SwitchMultiport = packageName + "SwitchMultiport";

    public static Optional<MathAssignmentExpressionSymbol> getStatementForPort(EMAComponentInstanceSymbol component, String portName) {
        Optional<MathAssignmentExpressionSymbol> res = preDefinedStatement(component, portName);

        if(!res.isPresent())
            res = calculateStatementForPort(component, portName);
        return res;
    }

    private static Optional<MathAssignmentExpressionSymbol> preDefinedStatement(EMAComponentInstanceSymbol component, String portName) {
        MathAssignmentExpressionSymbol res = null;

        String componentTypeName = component.getComponentType().getReferencedComponent().get().getPackageName()
         + "." + component.getComponentType().getName();
        if (componentTypeName.equals(Sum) || componentTypeName.equals(Multiplication)) {
            String operator = componentTypeName.equals(Sum) ? "+" :
                    componentTypeName.equals(Multiplication) ? "*" : "";
            ASTResolution resolution = component.getResolutionDeclarationSymbol("n").getASTResolution();
            double n = 0;
            if (resolution instanceof ASTUnitNumberResolution) {
                n = ((ASTUnitNumberResolution) resolution).getNumber().get();
            } else
                Log.error("0x695463987 Sum Resolution not set");

            res = calculateArithmeticOverArray((int) n, "in1", "out1", operator);
        }

        return Optional.ofNullable(res);
    }

    private static MathAssignmentExpressionSymbol calculateArithmeticOverArray(int n, String inportName, String outportName, String operator) {
        MathAssignmentExpressionSymbol res = new MathAssignmentExpressionSymbol();
        res.setNameOfMathValue(outportName);
        res.setAssignmentOperator(new MathAssignmentOperator("="));

        MathExpressionSymbol currentExpression = new MathNameExpressionSymbol(inportName + "(1)");
        for (int i = 2; i <= n; i++) {
            MathNameExpressionSymbol rightExpression = new MathNameExpressionSymbol(inportName + "(" + i + ")");
            MathArithmeticExpressionSymbol add = new MathArithmeticExpressionSymbol();
            add.setOperator(operator);
            add.setLeftExpression(currentExpression);
            add.setRightExpression(rightExpression);
            currentExpression = add;
        }

        res.setExpressionSymbol(currentExpression);
        return res;
    }

    private static Optional<MathAssignmentExpressionSymbol> calculateStatementForPort(EMAComponentInstanceSymbol component, String portName) {
        Optional<MathStatementsSymbol> mathStatements = component.getSpannedScope().<MathStatementsSymbol>resolve("MathStatements", MathStatementsSymbol.KIND);
        if (!mathStatements.isPresent())
            return Optional.empty();


        MathAssignmentExpressionSymbol res = null;
        // Copy to not alter symbol table
        MathStatementsSymbol mathStatementsSymbolCopy = CopyMathExpressionSymbol.copy(mathStatements.get());
        ListIterator<MathExpressionSymbol> iter = mathStatementsSymbolCopy.getMathExpressionSymbols()
                .listIterator(mathStatementsSymbolCopy.getMathExpressionSymbols().size());
        while (iter.hasPrevious()) {
            MathExpressionSymbol statement = iter.previous();
            if (statement.isAssignmentExpression()) {
                MathAssignmentExpressionSymbol assignmentStatement = (MathAssignmentExpressionSymbol) statement;

                if (assignmentStatement.getNameOfMathValue().equals(portName)) {
                    res = assignmentStatement;
                } else if (res != null) {
                    // TODO replace
                    res = res;
                }
            } else
                Log.error("0x654987 not yet supported");
        }

        return Optional.ofNullable(res);
    }
}
