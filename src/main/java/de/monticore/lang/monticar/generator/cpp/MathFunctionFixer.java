package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.math.math._symboltable.matrix.*;
import de.monticore.lang.monticar.generator.MathCommand;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.cpp.symbols.MathChainedExpression;
import de.monticore.lang.monticar.generator.cpp.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

/**
 * @author Sascha Schneiders
 */
public class MathFunctionFixer {
    public static void fixMathFunctions(MathExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        boolean notHandled = true;
        if (mathExpressionSymbol == null) {
            notHandled = false;
        } else if (mathExpressionSymbol.isAssignmentExpression()) {
            fixMathFunctions((MathAssignmentExpressionSymbol) mathExpressionSymbol, bluePrintCPP);
            notHandled = false;
        } else if (mathExpressionSymbol.isMatrixExpression()) {
            fixMathFunctions((MathMatrixExpressionSymbol) mathExpressionSymbol, bluePrintCPP);
            notHandled = false;
        } else if (mathExpressionSymbol.isArithmeticExpression()) {
            fixMathFunctions((MathArithmeticExpressionSymbol) mathExpressionSymbol, bluePrintCPP);
            notHandled = false;
        } else if (mathExpressionSymbol.isForLoopExpression()) {
            fixMathFunctions((MathForLoopExpressionSymbol) mathExpressionSymbol, bluePrintCPP);
            notHandled = false;
        } else if (mathExpressionSymbol.isCompareExpression()) {
            fixMathFunctions((MathCompareExpressionSymbol) mathExpressionSymbol, bluePrintCPP);
            notHandled = false;
        } else if (mathExpressionSymbol.isAssignmentDeclarationExpression()) {
            fixMathFunctions((MathValueSymbol) mathExpressionSymbol, bluePrintCPP);
            notHandled = false;
        } else if (mathExpressionSymbol.isParenthesisExpression()) {
            fixMathFunctions((MathParenthesisExpressionSymbol) mathExpressionSymbol, bluePrintCPP);
            notHandled = false;
        } else if (mathExpressionSymbol.isConditionalsExpression()) {
            fixMathFunctions((MathConditionalExpressionsSymbol) mathExpressionSymbol, bluePrintCPP);
            notHandled = false;
        } else if (mathExpressionSymbol.isConditionalExpression()) {
            fixMathFunctions((MathConditionalExpressionSymbol) mathExpressionSymbol, bluePrintCPP);
            notHandled = false;
        } else if (mathExpressionSymbol.isPreOperatorExpression()) {
            fixMathFunctions((MathPreOperatorExpressionSymbol) mathExpressionSymbol, bluePrintCPP);
            notHandled = false;
        } else if (mathExpressionSymbol.isValueExpression()) {
            MathValueExpressionSymbol mathValueExpressionSymbol = (MathValueExpressionSymbol) mathExpressionSymbol;
            if (mathValueExpressionSymbol.isNumberExpression()) {
                notHandled = false;
            } else if (((MathValueExpressionSymbol) mathExpressionSymbol).isNameExpression()) {
                notHandled = false;
            }
        } else if (mathExpressionSymbol.getExpressionID() == MathChainedExpression.ID) {
            fixMathFunctions((MathChainedExpression) mathExpressionSymbol, bluePrintCPP);
            notHandled = false;
        } else if (mathExpressionSymbol.getExpressionID() == MathStringExpression.ID) {
            notHandled = false;
            //MathStringExpressions should be correct
        }
        if (notHandled) {
            Log.info(mathExpressionSymbol.getClass().getName(), "Symbol name:");
            Log.info(mathExpressionSymbol.getTextualRepresentation(), "Symbol:");
            Log.error("Case not handled!");
        }
    }

    public static void fixMathFunctions(MathPreOperatorExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        fixMathFunctions(mathExpressionSymbol.getMathExpressionSymbol(), bluePrintCPP);
    }

    public static void fixMathFunctions(MathChainedExpression mathChainedExpression, BluePrintCPP bluePrintCPP) {
        if (mathChainedExpression.getFirstExpressionSymbol().isMatrixExpression()) {
            MathMatrixExpressionSymbol mathMatrixExpressionSymbol = (MathMatrixExpressionSymbol) mathChainedExpression.getFirstExpressionSymbol();
            if (mathMatrixExpressionSymbol.isMatrixAccessExpression()) {
                if (mathChainedExpression.getSecondExpressionSymbol().getExpressionID() == MathStringExpression.ID) {
                    MathStringExpression mathStringExpression = (MathStringExpression) mathChainedExpression.getSecondExpressionSymbol();
                    if (mathStringExpression.getText().contains("-1"))
                        return;
                }
            }
        }
        fixMathFunctions(mathChainedExpression.getFirstExpressionSymbol(), bluePrintCPP);
    }

    public static void fixMathFunctions(MathConditionalExpressionsSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        fixMathFunctions(mathExpressionSymbol.getIfConditionalExpression(), bluePrintCPP);
        for (MathConditionalExpressionSymbol mathConditionalExpressionSymbol : mathExpressionSymbol.getIfElseConditionalExpressions())
            fixMathFunctions(mathConditionalExpressionSymbol, bluePrintCPP);
        if (mathExpressionSymbol.getElseConditionalExpression().isPresent())
            fixMathFunctions(mathExpressionSymbol.getElseConditionalExpression().get(), bluePrintCPP);
    }

    public static void fixMathFunctions(MathConditionalExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        if (mathExpressionSymbol.getCondition().isPresent())
            fixMathFunctions(mathExpressionSymbol.getCondition().get(), bluePrintCPP);
        for (MathExpressionSymbol bodyExpression : mathExpressionSymbol.getBodyExpressions())
            fixMathFunctions(bodyExpression, bluePrintCPP);
    }

    public static void fixMathFunctions(MathValueSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        bluePrintCPP.getMathInformationRegister().addVariable(mathExpressionSymbol);
        if (mathExpressionSymbol.getValue() != null)
            fixMathFunctions(mathExpressionSymbol.getValue(), bluePrintCPP);
    }

    public static void fixMathFunctions(MathAssignmentExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        if (mathExpressionSymbol.getMathMatrixAccessOperatorSymbol() != null && fixForLoopAccess(mathExpressionSymbol.getNameOfMathValue(), bluePrintCPP)) {
            for (MathMatrixAccessSymbol mathMatrixAccessSymbol : mathExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
                //fixMathFunctionsForLoopAccess(mathMatrixAccessSymbol, bluePrintCPP);
                fixMathFunctions(mathMatrixAccessSymbol, bluePrintCPP);

        }
        fixMathFunctions(mathExpressionSymbol.getExpressionSymbol(), bluePrintCPP);
    }

    //TODO find function which does not pass fixMathFunction/debug MathSumCommand
    public static void fixMathFunctions(MathMatrixExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        if (mathExpressionSymbol.isMatrixNameExpression()) {
            fixMathFunctions((MathMatrixNameExpressionSymbol) mathExpressionSymbol, bluePrintCPP);
        } else if (mathExpressionSymbol.isMatrixVectorExpression()) {
            fixMathFunctions((MathMatrixVectorExpressionSymbol) mathExpressionSymbol, bluePrintCPP);
        } else if (mathExpressionSymbol.isMatrixAccessExpression()) {
            fixMathFunctions((MathMatrixAccessSymbol) mathExpressionSymbol, bluePrintCPP);
        } else if (mathExpressionSymbol.isMatrixArithmeticExpression()) {
            fixMathFunctions((MathMatrixArithmeticExpressionSymbol) mathExpressionSymbol, bluePrintCPP);
        } else if (mathExpressionSymbol.isValueExpression()) {
            //Nothing to do here
        } else {
            Log.info(mathExpressionSymbol.getClass().getName(), "Symbol name:");
            Log.info(mathExpressionSymbol.getTextualRepresentation(), "Symbol:");
            Log.error("MathConverter Case not handled!");
        }
    }

    public static void fixMathFunctions(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        fixMathFunctions(mathExpressionSymbol.getLeftExpression(), bluePrintCPP);
        if (mathExpressionSymbol.getRightExpression() != null)
            fixMathFunctions(mathExpressionSymbol.getRightExpression(), bluePrintCPP);
    }

    public static void fixMathFunctions(MathMatrixNameExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        String name = PortSymbol.getNameWithoutArrayBracketPart(mathExpressionSymbol.getNameToAccess());
        Variable variable = bluePrintCPP.getVariable(name).orElse(null);
        //change () to [] if it is a variable and no function
        if (variable != null && variable.isArray()) {
            Log.info(name, "Fixing variable array access:");
            mathExpressionSymbol.getMathMatrixAccessOperatorSymbol().setAccessStartSymbol("[");
            mathExpressionSymbol.getMathMatrixAccessOperatorSymbol().setAccessEndSymbol("]");
            for (MathMatrixAccessSymbol mathMatrixAccessSymbol : mathExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols()) {
                fixMathFunctionsForLoopAccess(mathMatrixAccessSymbol, bluePrintCPP);
            }
        } else {
            MathCommand mathCommand = bluePrintCPP.getMathCommandRegister().getMathCommand(mathExpressionSymbol.getNameToAccess());
            if (mathCommand != null) {
                if (MathConverter.curBackend.getBackendName().equals("OctaveBackend"))
                    bluePrintCPP.addAdditionalIncludeString("Helper");
                mathCommand.convertAndSetTargetLanguageName(mathExpressionSymbol, bluePrintCPP);
            }
            if (fixForLoopAccess(mathExpressionSymbol, variable, bluePrintCPP)) {
                for (MathMatrixAccessSymbol mathMatrixAccessSymbol : mathExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols()) {
                    //fixMathFunctionsForLoopAccess(mathMatrixAccessSymbol, bluePrintCPP);
                    fixMathFunctions(mathMatrixAccessSymbol, bluePrintCPP);
                }
            } else {
                for (MathMatrixAccessSymbol mathMatrixAccessSymbol : mathExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols()) {
                    fixMathFunctions(mathMatrixAccessSymbol, bluePrintCPP);
                }
            }
        }
    }

    public static void fixMathFunctions(MathMatrixAccessSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        if (mathExpressionSymbol.getMathExpressionSymbol().isPresent()) {
            fixMathFunctions(mathExpressionSymbol.getMathExpressionSymbol().get(), bluePrintCPP);
            MathExpressionSymbol mathExp = mathExpressionSymbol.getMathExpressionSymbol().get();
            /*if (mathExp.getExpressionID() != MathChainedExpression.ID && mathExp.getExpressionID() != MathStringExpression.ID) {
                mathExpressionSymbol.setMathExpressionSymbol(new MathChainedExpression(mathExp, new MathStringExpression("-1")));
            }*/
            //FOR LOOP VARIABLE SUBTRACTION
              /*  Variable var = bluePrintCPP.getMathInformationRegister().getVariable(mathExp.getTextualRepresentation());
                if (var != null)
                    Log.info(var.getName(), "fixMathFunctions VAR:");
                else
                    Log.info(mathExp.getTextualRepresentation(), "VAR not found:");
                if (var != null && var.isForLoopVariable())
                */
        }
    }

    public static void fixMathFunctionsForLoopAccess(MathMatrixAccessSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        if (mathExpressionSymbol.getMathExpressionSymbol().isPresent()) {
            fixMathFunctions(mathExpressionSymbol.getMathExpressionSymbol().get(), bluePrintCPP);
            MathExpressionSymbol mathExp = mathExpressionSymbol.getMathExpressionSymbol().get();
            //mathExpressionSymbol.setMathExpressionSymbol(mathExp);
            if (!(mathExp instanceof MathChainedExpression))
                mathExpressionSymbol.setMathExpressionSymbol(new MathChainedExpression(mathExp, new MathStringExpression("-1", null)));
            /*if (mathExp.getExpressionID() != MathChainedExpression.ID && mathExp.getExpressionID() != MathStringExpression.ID) {

                mathExpressionSymbol.setMathExpressionSymbol(new MathChainedExpression(mathExp, new MathStringExpression("-1")));
            }*/

            //FOR LOOP VARIABLE SUBTRACTION
              /*  Variable var = bluePrintCPP.getMathInformationRegister().getVariable(mathExp.getTextualRepresentation());
                if (var != null)
                    Log.info(var.getName(), "fixMathFunctions VAR:");
                else
                    Log.info(mathExp.getTextualRepresentation(), "VAR not found:");
                if (var != null && var.isForLoopVariable())
                */
        }

    }


    public static void fixMathFunctions(MathParenthesisExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        fixMathFunctions(mathExpressionSymbol.getMathExpressionSymbol(), bluePrintCPP);
    }

    public static void fixMathFunctions(MathArithmeticExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        fixMathFunctions(mathExpressionSymbol.getLeftExpression(), bluePrintCPP);
        fixMathFunctions(mathExpressionSymbol.getRightExpression(), bluePrintCPP);
    }

    public static void fixMathFunctions(MathForLoopExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        ComponentConverter.currentBluePrint.getMathInformationRegister().addVariable(new Variable(mathExpressionSymbol.getForLoopHead().getNameLoopVariable(), Variable.FORLOOPINFO));
        fixMathFunctions(mathExpressionSymbol.getForLoopHead().getMathExpression(), bluePrintCPP);
        for (MathExpressionSymbol mathExpressionSymbol1 : mathExpressionSymbol.getForLoopBody())
            fixMathFunctions(mathExpressionSymbol1, bluePrintCPP);
    }

    public static void fixMathFunctions(MathCompareExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        fixMathFunctions(mathExpressionSymbol.getLeftExpression(), bluePrintCPP);
        fixMathFunctions(mathExpressionSymbol.getRightExpression(), bluePrintCPP);
    }

    public static void fixMathFunctions(MathMatrixVectorExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        fixMathFunctions(mathExpressionSymbol.getStart(), bluePrintCPP);
        if (mathExpressionSymbol.getStep().isPresent())
            fixMathFunctions(mathExpressionSymbol.getStep().get(), bluePrintCPP);
        fixMathFunctions(mathExpressionSymbol.getEnd(), bluePrintCPP);
    }

    public static boolean fixForLoopAccess(MathMatrixNameExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        Variable variable = MathConverter.getVariableFromBluePrint(mathExpressionSymbol, bluePrintCPP);
        return fixForLoopAccess(mathExpressionSymbol, variable, bluePrintCPP);
    }

    public static boolean fixForLoopAccess(String nameToAccess, BluePrintCPP bluePrintCPP) {
        Variable variable = MathConverter.getVariableFromBluePrint(nameToAccess, bluePrintCPP);
        if (variable == null)
            variable = bluePrintCPP.getMathInformationRegister().getVariable(nameToAccess);
        return fixForLoopAccess(nameToAccess, variable, bluePrintCPP);
    }

    public static boolean fixForLoopAccess(MathMatrixNameExpressionSymbol mathExpressionSymbol, Variable variable, BluePrintCPP bluePrintCPP) {
        return fixForLoopAccess(mathExpressionSymbol.getNameToAccess(), variable, bluePrintCPP);
    }

    public static boolean fixForLoopAccess(String nameToAccess, Variable variable, BluePrintCPP bluePrintCPP) {
        MathCommand mathCommand = bluePrintCPP.getMathCommandRegister().getMathCommand(nameToAccess);
        if ((mathCommand == null) && (variable != null) && (variable.getVariableType() != null) && (variable.getVariableType().getTypeNameTargetLanguage() != null)) {
            String type = variable.getVariableType().getTypeNameTargetLanguage();
            return isTargetLanguageMatrixType(type);
        }
        return false;
    }

    private static boolean isTargetLanguageMatrixType(String type) {
        boolean isMatrixType = false;
        String matType = MathConverter.curBackend.getMatrixTypeName();
        String colVecType = MathConverter.curBackend.getColumnVectorTypeName();
        String cubeType = MathConverter.curBackend.getCubeTypeName();
        if (matType != null)
            isMatrixType |= type.contentEquals(matType);
        if (colVecType != null)
            isMatrixType |= type.contentEquals(colVecType);
        if (cubeType != null)
            isMatrixType |= type.contentEquals(cubeType);
        return isMatrixType;
    }
}