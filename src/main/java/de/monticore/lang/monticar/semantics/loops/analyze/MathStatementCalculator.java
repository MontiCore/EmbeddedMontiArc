/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathAssignmentOperator;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.copy.CopyMathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTResolution;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.se_rwth.commons.logging.Log;

import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Optional;

public class MathStatementCalculator {

    // TODO

    private static final String packageName = "de.monticore.lang.monticar.semantics.basicLibrary";
    private static final String packageNameWithDot = packageName + ".";
    private static final String And = packageNameWithDot + "And";
    private static final String Constant = packageNameWithDot + "Constant";
    private static final String Delay = packageNameWithDot + "Delay";
    private static final String Derivative = packageNameWithDot + "Derivative";
    private static final String Division = packageNameWithDot + "Division";
    private static final String Equals = packageNameWithDot + "Equals";
    private static final String Gain = packageNameWithDot + "Gain";
    private static final String Greater = packageNameWithDot + "Greater";
    private static final String GreaterEquals = packageNameWithDot + "GreaterEquals";
    private static final String Integration = packageNameWithDot + "Integration";
    private static final String Less = packageNameWithDot + "Less";
    private static final String LookUp = packageNameWithDot + "LookUp";
    private static final String Max = packageNameWithDot + "Max";
    private static final String Memory = packageNameWithDot + "Memory";
    private static final String Min = packageNameWithDot + "Min";
    private static final String MinusPlus = packageNameWithDot + "MinusPlus";
    private static final String Mod = packageNameWithDot + "Mod";
    private static final String MultDiv = packageNameWithDot + "MultDiv";
    private static final String Multiplication = packageNameWithDot + "Multiplication";
    private static final String Not = packageNameWithDot + "Not";
    private static final String Or = packageNameWithDot + "Or";
    private static final String PlusMinus = packageNameWithDot + "PlusMinus";
    private static final String PlusMinusPlus = packageNameWithDot + "PlusMinusPlus";
    private static final String Saturation = packageNameWithDot + "Saturation";
    private static final String Smaller = packageNameWithDot + "Smaller";
    private static final String SmallerEquals = packageNameWithDot + "SmallerEquals";
    private static final String Sum = packageNameWithDot + "Sum";
    private static final String Switch = packageNameWithDot + "Switch";
    private static final String SwitchB = packageNameWithDot + "SwitchB";
    private static final String SwitchM = packageNameWithDot + "SwitchM";
    private static final String SwitchMultiport = packageNameWithDot + "SwitchMultiport";

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
                Log.error("0x695463987 Resolution of " + component.getFullName() + " is not set");

            res = calculateArithmeticOverArray((int) n, "in1", "out1", operator);
        } else if (componentTypeName.equals(Integration)) {
            // TODO different cases for loop or not in loop, for now assume in loop
            res = new MathAssignmentExpressionSymbol();
            res.setAssignmentOperator(new MathAssignmentOperator("="));
            res.setNameOfMathValue("in1");
            res.setPackageName(packageName);
            res.setFullName(Integration);

            MathMatrixNameExpressionSymbol diff = new MathMatrixNameExpressionSymbol("Derivative");
            diff.setPackageName(packageName);
            diff.setFullName(Integration + ".");

            MathMatrixAccessOperatorSymbol accessOperatorSymbol = new MathMatrixAccessOperatorSymbol();
            accessOperatorSymbol.setPackageName(packageName);
            accessOperatorSymbol.setFullName(Integration + ".");
            accessOperatorSymbol.setMathMatrixNameExpressionSymbol(diff);
            diff.setMathMatrixAccessOperatorSymbol(accessOperatorSymbol);

            MathNameExpressionSymbol out1 = new MathNameExpressionSymbol("out1");
            out1.setPackageName(packageName);
            out1.setFullName(Integration + ".");

            MathMatrixAccessSymbol accessSymbol = new MathMatrixAccessSymbol(out1);
            accessSymbol.setPackageName(packageName);
            accessSymbol.setFullName(Integration + ".");

            List<MathMatrixAccessSymbol> accessSymbols = new LinkedList<>();
            accessSymbols.add(accessSymbol);
            accessOperatorSymbol.setMathMatrixAccessSymbols(accessSymbols);

            res.setExpressionSymbol(diff);
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
