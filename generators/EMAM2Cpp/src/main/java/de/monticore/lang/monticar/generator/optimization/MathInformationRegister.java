/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.EMAMBluePrint;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * This class stores information of already encountered math information
 *
 */
public class MathInformationRegister {
    protected List<Variable> variables = new ArrayList<>();//contains in implementation Math section declared Variables
    protected List<MathValueSymbol> mathValueSymbols = new ArrayList<>();
    protected EMAMBluePrint bluePrint;

    public MathInformationRegister(EMAMBluePrint bluePrint) {
        this.bluePrint = bluePrint;
    }

    public MathValueSymbol getMathValueSymbol(String name) {
        for (MathValueSymbol mathValueSymbol : mathValueSymbols) {
            if (mathValueSymbol.getName().equals(name)) {
                return mathValueSymbol;
            }
        }
        return null;
    }

    public int getAmountRows(String name) {
        return getAmount(name, 0);
    }

    public int getAmountRows(String name, MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol) {
        boolean firstDoubleDot = false, secondDoubleDot = false;
        int result = 0;
        if (mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols().size() == 2) {
            firstDoubleDot = mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols().get(0).isDoubleDot();
            secondDoubleDot = mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols().get(1).isDoubleDot();
        }

        //Check "doubleDots" !!!
        if (secondDoubleDot)
            result = 1;
        else {
            result = getAmount(name, 0);
        }
        return result;
    }

    public int getAmount(String name) {
        MathValueSymbol mathValueSymbol = getMathValueSymbol(name);
        int result = 0;
        if (mathValueSymbol != null) {
            String numberString = mathValueSymbol.getType().getDimensions().get(1).getTextualRepresentation();
            try {
                result = Integer.valueOf(numberString);
            } catch (Exception ex) {
                // TODO resolve name return bluePrint.
            }
        } else {
            Variable var = getVariable(name);

            if (var != null) {
                try {
                    result = Integer.valueOf(var.getDimensionalInformation().get(1));
                } catch (Exception ex) {
                    // TODO resolve name return bluePrint.
                    Log.info(name, "Name:");
                    //ex.printStackTrace();
                    //Log.error(var.getDimensionalInformation().get(1));
                    result = 1;
                }
            }
        }
        return result;
    }

    public int getAmount(String name, int dimension) {
        MathValueSymbol mathValueSymbol = getMathValueSymbol(name);
        int result = 0;
        if (mathValueSymbol != null) {
            String numberString = mathValueSymbol.getType().getDimensions().get(dimension).getTextualRepresentation();
            Log.info(name, "Name:");
            Log.info(mathValueSymbol.getType().getDimensions().get(0).getTextualRepresentation(), "getAmount");
            Log.info(mathValueSymbol.getType().getDimensions().get(1).getTextualRepresentation(), "getAmount");
            try {
                result = Integer.valueOf(numberString);
            } catch (Exception ex) {
                // TODO resolve name return bluePrint.
            }
        } else {
            Variable var = getVariable(name);
            if (var != null) {
                try {
                    Log.info(name, "Name:");
                    Log.info(var.getDimensionalInformation().get(0), "getAmount");
                    Log.info(var.getDimensionalInformation().get(1), "getAmount");

                    result = Integer.valueOf(var.getDimensionalInformation().get(dimension));
                } catch (Exception ex) {
                    // TODO resolve name return bluePrint.
                    Log.info("getAmount " + ex.getMessage(), "Not handled:");
                    result = 1;
                }
            } else
                Log.info(name, "Not found:");
        }
        return result;
    }

    public int getAmountColumns(String name) {
        return getAmount(name, 1);
    }

    public int getAmountColumns(String name, MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol) {
        boolean firstDoubleDot = false, secondDoubleDot = false;
        int result = 0;
        if (mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols().size() == 2) {
            firstDoubleDot = mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols().get(0).isDoubleDot();
            secondDoubleDot = mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols().get(1).isDoubleDot();
        }
        if (firstDoubleDot)
            result = 1;
        else {
            result = getAmount(name, 1);
        }
        return result;
    }

    public List<Variable> getVariables() {
        return variables;
    }

    public void setVariables(List<Variable> variables) {
        this.variables = variables;
    }

    public void addVariable(Variable variable) {

        if (this.variables.contains(variable)) {

        } else {
            Log.info("v: " + variable.getName(), "addVariable");
            this.variables.add(variable);
        }
    }

    public void addVariable(MathValueSymbol mathValueSymbol) {
        Log.info("mathValueSymbol: " + mathValueSymbol.getTextualRepresentation(), "addVariable");
        Variable var = new Variable(mathValueSymbol.getName(), Variable.VARIABLE);
        var.setTypeNameTargetLanguage(TypeConverter.getVariableTypeNameForMathLanguageTypeName(mathValueSymbol.getType()));
        for (MathExpressionSymbol dimension : mathValueSymbol.getType().getDimensions())
            var.addDimensionalInformation(dimension.getTextualRepresentation());
        this.variables.add(var);
        var.addProperties(mathValueSymbol.getType().getProperties());
        var.addAdditionalInformation(Variable.FROMMATH);
        this.mathValueSymbols.add(mathValueSymbol);
    }

    public Variable getVariable(String name) {
        for (Variable v : variables)
            if (v.getName().equals(name))
                return v;
        for (Variable v : variables)
            if (v.getNameTargetLanguageFormat().equals(name))
                return v;
        return null;
    }

    public static String getVariableInitName(Variable v, EMAMBluePrintCPP bluePrint) {
        /*for (Variable v2 : bluePrint.getVariables()) {
            if (v2.isArray() && PortConverter.getPortNameWithoutArrayBracketPart(v2.getName()).equals(v.getName())) {
                if (!v.getName().endsWith("]")) {
                    return v.getName() + "[0]";
                }
            }
        }*/
        return v.getNameTargetLanguageFormat();
    }

    public Optional<Double> tryGetDoubleValue(MathExpressionSymbol symbol) {
        Optional<Double> result = Optional.empty();
        if (symbol.isValueExpression()) {
            if (((MathValueExpressionSymbol) symbol).isNumberExpression()) {
                // is number
                MathNumberExpressionSymbol numberSymbol = (MathNumberExpressionSymbol) symbol;
                result = Optional.of(numberSymbol.getValue().getRealNumber().doubleValue());
            } else if (symbol instanceof MathValueSymbol) {
                result = tryGetDoubleValue(((MathValueSymbol) symbol).getValue());
            } else if (symbol instanceof MathNameExpressionSymbol) {
                Optional<MathValueSymbol> resolvedSymbol = symbol.getEnclosingScope().resolve(((MathNameExpressionSymbol) symbol).getNameToResolveValue(), MathValueSymbol.KIND);
                if (resolvedSymbol.isPresent()) {
                    result = tryGetDoubleValue(resolvedSymbol.get());
                }
            }
        }
        return result;
    }

    /**
     * Searches MathValueSymbol in list and returns first matching symbol which contains the type declaration information
     *
     * @param symbol Symbol for which type information should be collected
     * @return returns the first type declaration as symbol
     */
    public MathValueSymbol getFullTypeInformation(MathValueSymbol symbol) {
        MathValueSymbol result = getMathValueSymbol(symbol.getName());
        if (result == null) {
            result = symbol;
            // ask symbol table
            result = (MathValueSymbol) symbol.getEnclosingScope().resolve(symbol.getName(), symbol.getKind()).orElse(null);
        }
        return result;
    }

    /**
     * Variables are replaced by their definition.
     *
     * @param nonAtomarExpression Expression in which the variables will be substituted
     * @param atomarValueName     Defines the name of the atomar variable after which no more substitution will be made
     * @return MathExpressionSymbol containing the replaces variables
     */
    public MathExpressionSymbol resolveMathExpressionToAtomarExpression(MathExpressionSymbol nonAtomarExpression, String atomarValueName) {
        MathExpressionSymbol result;
        if (nonAtomarExpression.isArithmeticExpression()) {
            // recursively substitute expressions
            MathArithmeticExpressionSymbol arithmeticSubstituted = (MathArithmeticExpressionSymbol) nonAtomarExpression;
            MathArithmeticExpressionSymbol arithmeticResult = new MathArithmeticExpressionSymbol();
            arithmeticResult.setMathOperator(arithmeticSubstituted.getOperator());
            arithmeticResult.setLeftExpression(resolveMathExpressionToAtomarExpression(arithmeticSubstituted.getLeftExpression(), atomarValueName));
            arithmeticResult.setRightExpression(resolveMathExpressionToAtomarExpression(arithmeticSubstituted.getRightExpression(), atomarValueName));
            result = arithmeticResult;
        } else if (nonAtomarExpression instanceof MathValueSymbol) {
            // substitute value expr by assigned expr
            MathValueSymbol valueExpr = (MathValueSymbol) nonAtomarExpression;
            result = getSubstituteByName(valueExpr.getName(), nonAtomarExpression, atomarValueName);
        } else if (nonAtomarExpression instanceof MathMatrixNameExpressionSymbol) {
            MathMatrixNameExpressionSymbol sourceExpr = (MathMatrixNameExpressionSymbol) nonAtomarExpression;
            MathMatrixNameExpressionSymbol targetExpr;
            // modifiy access name
            String accessName = getSubstituteByName(sourceExpr.getNameToAccess(), nonAtomarExpression, atomarValueName).getTextualRepresentation();
            if (sourceExpr.isMathMatrixAccessOperatorSymbolPresent() && !(accessName.contains("("))) {
                targetExpr = new MathMatrixNameExpressionSymbol(String.format("(%s)", accessName));
                // modify access operator
                MathMatrixAccessOperatorSymbol sourceAccessOperator = sourceExpr.getMathMatrixAccessOperatorSymbol();
                MathMatrixAccessOperatorSymbol targetAccessOperator = new MathMatrixAccessOperatorSymbol();
                targetAccessOperator.setMathMatrixAccessSymbols(sourceAccessOperator.getMathMatrixAccessSymbols());
                targetAccessOperator.setAccessStartSymbol(".at(");
                targetExpr.setMathMatrixAccessOperatorSymbol(targetAccessOperator);
                result = targetExpr;
            } else {
                // try to resolve access
                MathExpressionSymbol accessOperator = resolveMathExpressionToAtomarExpression(sourceExpr.getMathMatrixAccessOperatorSymbol(), atomarValueName);
                targetExpr = sourceExpr;
                targetExpr.setMathMatrixAccessOperatorSymbol((MathMatrixAccessOperatorSymbol) accessOperator);
                result = targetExpr;
            }
        } else if (nonAtomarExpression instanceof IMathNamedExpression) {
            // substitute value expr by assigned expr
            result = getSubstituteByName(((IMathNamedExpression) nonAtomarExpression).getNameToAccess(), nonAtomarExpression, atomarValueName);
        } else if (nonAtomarExpression instanceof MathMatrixAccessOperatorSymbol) {
            MathMatrixAccessOperatorSymbol sourceExpr = (MathMatrixAccessOperatorSymbol) nonAtomarExpression;
            MathMatrixAccessOperatorSymbol targetExpression = new MathMatrixAccessOperatorSymbol();
            targetExpression.setMathMatrixAccessSymbols(new ArrayList<>());
            boolean hasResult = true;
            for (int i = 0; i < sourceExpr.getMathMatrixAccessSymbols().size(); i++) {
                MathMatrixAccessSymbol newAccess;
                if (sourceExpr.getMathMatrixAccessSymbol(i).isPresent()) {
                    MathExpressionSymbol newAccessExpr = resolveMathExpressionToAtomarExpression(sourceExpr.getMathMatrixAccessSymbol(i).get(), atomarValueName);
                    newAccess = new MathMatrixAccessSymbol();
                    newAccess.setMathExpressionSymbol(newAccessExpr);
                } else {
                    newAccess = null;
                    hasResult = false;
                    break;
                }
                targetExpression.addMathMatrixAccessSymbol(newAccess);
            }
            if (!hasResult)
                result = sourceExpr;
            else
                result = targetExpression;
        } else if (nonAtomarExpression instanceof MathMatrixAccessSymbol) {
            MathMatrixAccessSymbol source = (MathMatrixAccessSymbol) nonAtomarExpression;
            MathMatrixAccessSymbol target = new MathMatrixAccessSymbol();
            MathExpressionSymbol mathExpr = resolveMathExpressionToAtomarExpression(source.getMathExpressionSymbol().get(), atomarValueName);
            target.setMathExpressionSymbol(mathExpr);
            result = target;
        } else {
            result = nonAtomarExpression;
        }
        return result;
    }

    private MathExpressionSymbol getSubstituteByName(String name, MathExpressionSymbol expr, String atomarValueName) {
        MathExpressionSymbol result;
        MathValueSymbol declaration = getMathValueSymbol(name);
        if ((declaration != null) && (declaration.getValue() != null) && (!name.contentEquals(atomarValueName)))
            result = declaration.getValue();
        else
            result = expr;
        return result;
    }
}
