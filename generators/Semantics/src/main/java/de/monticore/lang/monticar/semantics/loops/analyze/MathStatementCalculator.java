/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.CopyEMAMMathExpressionSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTResolution;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.semantics.util.math.NameReplacer;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class MathStatementCalculator {

    // TODO

    public static final String packageName = "de.monticore.lang.monticar.semantics.basicLibrary";
    public static final String packageNameWithDot = packageName + ".";
    public static final String And = packageNameWithDot + "And";
    public static final String Constant = packageNameWithDot + "Constant";
    public static final String Delay = packageNameWithDot + "Delay";
    public static final String Derivative = packageNameWithDot + "Derivative";
    public static final String Division = packageNameWithDot + "Division";
    public static final String Equals = packageNameWithDot + "Equals";
    public static final String Gain = packageNameWithDot + "Gain";
    public static final String Greater = packageNameWithDot + "Greater";
    public static final String GreaterEquals = packageNameWithDot + "GreaterEquals";
    public static final String Integration = packageNameWithDot + "Integration";
    public static final String Less = packageNameWithDot + "Less";
    public static final String LookUp = packageNameWithDot + "LookUp";
    public static final String Max = packageNameWithDot + "Max";
    public static final String Memory = packageNameWithDot + "Memory";
    public static final String Min = packageNameWithDot + "Min";
    public static final String MinusPlus = packageNameWithDot + "MinusPlus";
    public static final String Mod = packageNameWithDot + "Mod";
    public static final String MultDiv = packageNameWithDot + "MultDiv";
    public static final String Multiplication = packageNameWithDot + "Multiplication";
    public static final String Not = packageNameWithDot + "Not";
    public static final String Or = packageNameWithDot + "Or";
    public static final String PlusMinus = packageNameWithDot + "PlusMinus";
    public static final String PlusMinusPlus = packageNameWithDot + "PlusMinusPlus";
    public static final String Saturation = packageNameWithDot + "Saturation";
    public static final String Smaller = packageNameWithDot + "Smaller";
    public static final String SmallerEquals = packageNameWithDot + "SmallerEquals";
    public static final String Sum = packageNameWithDot + "Sum";
    public static final String Switch = packageNameWithDot + "Switch";
    public static final String SwitchB = packageNameWithDot + "SwitchB";
    public static final String SwitchM = packageNameWithDot + "SwitchM";
    public static final String SwitchMultiport = packageNameWithDot + "SwitchMultiport";

    public static Set<EMAMEquationSymbol> getStatementForPort(EMAPortInstanceSymbol port) {
        EMAComponentInstanceSymbol component = port.getComponentInstance();
        Set<EMAMEquationSymbol> res = preDefinedStatement(component, port);

//        if(res.isEmpty())
//            res = calculateStatementForPortSymbolic(component, port);
        return res;
    }

    private static Set<EMAMEquationSymbol> preDefinedStatement(EMAComponentInstanceSymbol component, EMAPortInstanceSymbol port) {
        EMAMEquationSymbol res = null;

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
        }

        return Collections.singleton(res);
    }

    private static EMAMEquationSymbol calculateArithmeticOverArray(int n, String inportName, String outportName, String operator) {
        EMAMEquationSymbol res = new EMAMEquationSymbol();
        res.setLeftExpression(new MathMatrixNameExpressionSymbol(outportName));

        MathExpressionSymbol currentExpression = new MathNameExpressionSymbol(inportName + "(1)");
        for (int i = 2; i <= n; i++) {
            MathNameExpressionSymbol rightExpression = new MathNameExpressionSymbol(inportName + "(" + i + ")");
            MathArithmeticExpressionSymbol add = new MathArithmeticExpressionSymbol();
            add.setOperator(operator);
            add.setLeftExpression(currentExpression);
            add.setRightExpression(rightExpression);
            currentExpression = add;
        }

        res.setRightExpression(currentExpression);
        return res;
    }

//    private static Set<EMAMEquationSymbol> calculateStatementForPort(EMAComponentInstanceSymbol component, EMAPortInstanceSymbol port) {
//        Optional<MathStatementsSymbol> mathStatements = component.getSpannedScope().<MathStatementsSymbol>resolve("MathStatements", MathStatementsSymbol.KIND);
//        if (!mathStatements.isPresent())
//            return Optional.empty();
//
//
//        EMAMEquationSymbol res = null;
//        // Copy to not alter symbol table
//        MathStatementsSymbol mathStatementsSymbolCopy = CopyMathExpressionSymbol.copy(mathStatements.get());
//        ListIterator<MathExpressionSymbol> iter = mathStatementsSymbolCopy.getMathExpressionSymbols()
//                .listIterator(mathStatementsSymbolCopy.getMathExpressionSymbols().size());
//        while (iter.hasPrevious()) {
//            MathExpressionSymbol statement = iter.previous();
//            if (statement.isAssignmentExpression()) {
//                MathAssignmentExpressionSymbol assignmentStatement = (MathAssignmentExpressionSymbol) statement;
//
//                if (assignmentStatement.getNameOfMathValue().equals(port.getName())) {
//                    res = assignmentStatement;
//                } else if (res != null) {
//                    // TODO replace
//                    res = res;
//                }
//            } else
//                Log.error("0x654987 not yet supported");
//        }
//
//        return Collections.singleton(res);
//    }

    private static Set<EMAMEquationSymbol> calculateStatementForPortSymbolic(EMAComponentInstanceSymbol component) {
        Optional<MathStatementsSymbol> mathStatements = component.getSpannedScope().resolve("MathStatements", MathStatementsSymbol.KIND);
        if (!mathStatements.isPresent())
            return new HashSet<>();

        MathStatementCalculator statementCalculator = new MathStatementCalculator();

        ListIterator<MathExpressionSymbol> iter = mathStatements.get().getMathExpressionSymbols()
                .stream().sorted(Comparator.comparingInt(c -> c.getSourcePosition().getLine()))
                .collect(Collectors.toList()).listIterator();

        while (iter.hasNext()) {
            MathExpressionSymbol expression = iter.next();
            if (expression.isAssignmentExpression()) {
                statementCalculator.handleExpression((MathAssignmentExpressionSymbol) expression);
            } else if (expression instanceof MathForLoopExpressionSymbol) {
                // TODO
                MathForLoopExpressionSymbol forLoopExpressionSymbol = (MathForLoopExpressionSymbol) expression;
                forLoopExpressionSymbol.toString();
            } else if (expression instanceof MathValueSymbol) {
                // TODO

            } else {
                Log.error("0x654987 not yet supported");
            }
        }

        return null;
    }


    private Set<MathAssignmentExpressionSymbol> system = new HashSet<>();
    private Set<String> variables = new HashSet<>();

    private void handleExpression(MathAssignmentExpressionSymbol expression) {
        MathAssignmentExpressionSymbol copy = CopyEMAMMathExpressionSymbol.copy(expression);
        String name = copy.getNameOfMathValue();
        incIndexOf(name);
        String currentName = getCurrentNameOf(name);
        NameReplacer replacer = new NameReplacer(s -> getCurrentNameOf(s));
        replacer.handle(copy);

        variables.add(currentName);
        system.add(copy);
    }

    private Map<String, Integer> lastIndexOf = new HashMap<>();
    private int getLastIndexOf(String var) {
        Integer index = lastIndexOf.get(var);
        if (index == null) return 0;
        return index;
    }
    private void incIndexOf(String var) {
        Integer index = lastIndexOf.get(var);
        if (index == null) index = -1;
        lastIndexOf.put(var, ++index);
    }
    private String getCurrentNameOf(String var) {
        int index = getLastIndexOf(var);
        return index != 0 ? var + "_" + index : var;
    }

}
