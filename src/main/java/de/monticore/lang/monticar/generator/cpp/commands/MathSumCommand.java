/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.OctaveHelper;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Overloaded syntax to more math convinient way:
 * sum(function, sum_variable, start_value, end_value)
 */
public class MathSumCommand extends MathCommand {

    private static final String SUM_SYNTAX_EXTENDED = "sum( EXPRESSION , SUM_VARIABLE , START_VALUE , END_VALUE )";

    private static final String CALC_SUM_METHOD_NAME = "calcSum";

    private static int sumCommandCounter = 0;

    public MathSumCommand() {
        setMathCommandName("sum");
        //setTargetCommand("LALALA");
    }

    @Override
    public void convert(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrint bluePrint) {
        String backendName = MathConverter.curBackend.getBackendName();
        if (backendName.equals("OctaveBackend")) {
            convertUsingOctaveBackend(mathExpressionSymbol, bluePrint);
        } else if (backendName.equals("ArmadilloBackend")) {
            convertUsingArmadilloBackend(mathExpressionSymbol, bluePrint);
        }

    }

    public void convertUsingOctaveBackend(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrint bluePrint) {
        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;

        mathMatrixNameExpressionSymbol.setNameToAccess("");

        String valueListString = "";
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
            MathFunctionFixer.fixMathFunctions(accessSymbol, (EMAMBluePrintCPP) bluePrint);
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<>());
        //OctaveHelper.getCallOctaveFunction(mathExpressionSymbol, "sum","Double", valueListString));
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression(OctaveHelper.getCallBuiltInFunction(mathExpressionSymbol, "Fsum", "Double", valueListString, "FirstResult", false, 1), mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));

        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        ((EMAMBluePrintCPP) bluePrint).addAdditionalUserIncludeStrings("octave/builtin-defun-decls");
        // error if using extended syntax here
        if (mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().size() == 4) {
            Log.error(String.format("Syntax: \"%s\" is not supported when using deprecated backend Octave", SUM_SYNTAX_EXTENDED));
        }
    }

    public void convertUsingArmadilloBackend(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrint bluePrint) {
        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;
        mathMatrixNameExpressionSymbol.setNameToAccess("");
        EMAMBluePrintCPP bluePrintCPP = (EMAMBluePrintCPP) bluePrint;
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
            MathFunctionFixer.fixMathFunctions(accessSymbol, bluePrintCPP);
        if (mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().size() == 1) {
            convertAccuSumImplementationArmadillo(mathMatrixNameExpressionSymbol, bluePrintCPP);
        } else if (mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().size() == 2) {
            convertSumImplementationArmadillo(mathMatrixNameExpressionSymbol, bluePrintCPP);
        } else if (mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().size() == 4) {
            MathMatrixAccessSymbol func = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(0);
            MathMatrixAccessSymbol sumVar = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(1);
            MathMatrixAccessSymbol sumStart = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(2);
            MathMatrixAccessSymbol sumEnd = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(3);
            convertExtendedSumImplementationArmadillo(mathMatrixNameExpressionSymbol, func, sumVar, sumStart, sumEnd, bluePrintCPP);
        } else {
            Log.error(String.format("No implementation found for sum operation: \"sum(%s)\". Possible syntax is \"sum( X )\", \"sum(X,dim)\" or \"%s\"", mathExpressionSymbol.getTextualRepresentation(), SUM_SYNTAX_EXTENDED));
        }
    }

    private void convertSumImplementationArmadillo(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, EMAMBluePrintCPP bluePrintCPP) {
        String valueListString = ExecuteMethodGenerator.generateExecuteCode(mathMatrixNameExpressionSymbol, new ArrayList<>());
        // correct index for armadillo
        valueListString = valueListString.substring(0, valueListString.length() - 1) + "-1)";
        MathStringExpression stringExpression = new MathStringExpression("sum" + valueListString, mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
    }

    /**
     * Implements the sum command using Armadillos accu command
     *
     * @param mathMatrixNameExpressionSymbol MathMatrixNameExpressionSymbol passed to convert
     * @param bluePrint                      EMAMBluePrint of current code generation
     * @see <a href="http://arma.sourceforge.net/docs.html#accu">Armadillo Documentation</a>
     */
    private void convertAccuSumImplementationArmadillo(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, EMAMBluePrintCPP bluePrint) {
        String valueListString = ExecuteMethodGenerator.generateExecuteCode(mathMatrixNameExpressionSymbol, new ArrayList<>());
        //OctaveHelper.getCallOctaveFunction(mathExpressionSymbol, "sum","Double", valueListString));
        MathStringExpression stringExpression = new MathStringExpression("accu" + valueListString, mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        bluePrint.addAdditionalUserIncludeStrings("HelperA"); // question: why? (CR)
    }

    /**
     * Implements a sum function with syntax "sum( EXPRESSION , SUM_VARIABLE , START_VALUE , END_VALUE )"
     * This syntax makes sum expressions easier to model.
     *
     * @param mathMatrixNameExpressionSymbol symbol to convert
     * @param func                           expression from which the sum is calculates
     * @param sumVar                         name of the sum variable
     * @param sumStart                       start value of the sum variable
     * @param sumEnd                         end value of the sum variable
     */
    private void convertExtendedSumImplementationArmadillo(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, MathMatrixAccessSymbol func, MathMatrixAccessSymbol sumVar, MathMatrixAccessSymbol sumStart, MathMatrixAccessSymbol sumEnd, EMAMBluePrintCPP bluePrint) {
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setAccessStartSymbol("");
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setAccessEndSymbol("");
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().clear();
        // create method
        Method calcSumMethod = getSumCalculationMethod(func, sumVar, sumStart, sumEnd, bluePrint);
        // create code string
        String code = calcSumMethod.getTargetLanguageMethodCall();
        MathStringExpression codeExpr = new MathStringExpression(code, mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().add(new MathMatrixAccessSymbol(codeExpr));
        // add method to bluePrint
        bluePrint.addMethod(calcSumMethod);
    }

    private Method getSumCalculationMethod(MathMatrixAccessSymbol func, MathMatrixAccessSymbol sumVar, MathMatrixAccessSymbol sumStart, MathMatrixAccessSymbol sumEnd, EMAMBluePrintCPP bluePrint) {
        // create new method
        Method method = getNewEmptySumCalculationMethod();
        // generate function code
        String f = ExecuteMethodGenerator.generateExecuteCode(func, new ArrayList<>());
        String varString = ExecuteMethodGenerator.generateExecuteCode(sumVar, new ArrayList<>());
        String start = ExecuteMethodGenerator.generateExecuteCode(sumStart, new ArrayList<>());
        String end = ExecuteMethodGenerator.generateExecuteCode(sumEnd, new ArrayList<>());
        // add loop var
        Variable loopVar = generateLoopVariable(varString, bluePrint);
        // parameters
        setParameters(method, bluePrint);
        // add instructions
        method.addInstruction(accumulatorInitialization());
        method.addInstruction(forLoopHeader(varString, start, end));
        method.addInstruction(forLoopBody(f));
        method.addInstruction(returnAccumulator());
        // add loopvar to children
        addLoopVarParamToMethod(method, loopVar, bluePrint);
        // delete loop var
        bluePrint.getMathInformationRegister().getVariables().remove(loopVar);
        return method;
    }

    private Method getNewEmptySumCalculationMethod() {
        sumCommandCounter++;
        Method method = new Method();
        method.setName(CALC_SUM_METHOD_NAME + sumCommandCounter);
        method.setReturnTypeName("double");
        return method;
    }

    private void setParameters(Method method, EMAMBluePrint bluePrint) {
        List<Variable> vars = bluePrint.getMathInformationRegister().getVariables();
        for (int i = 0; i < vars.size() - 2; i++) { // the last variable is the one we are assigning now
            method.addParameterUnique(vars.get(i));
        }
    }

    private Variable generateLoopVariable(String name, EMAMBluePrint bluePrint) {
        Variable loopVar = new Variable(name, Variable.FORLOOPINFO);
        loopVar.setVariableType(new VariableType("Integer", "int", ""));
        bluePrint.getMathInformationRegister().addVariable(loopVar);
        return loopVar;
    }

    private Instruction accumulatorInitialization() {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                return "  double res = 0; \n";
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }

    private Instruction forLoopHeader(String sumVar, String sumStart, String sumEnd) {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                return String.format("  for (int %s = %s; %s <= %s; %s++)\n", sumVar, sumStart, sumVar, sumEnd, sumVar);
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }

    private Instruction forLoopBody(String func) {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                return String.format("    res += %s;\n", func);
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }

    private Instruction returnAccumulator() {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                return "  return res;\n";
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }

    private void addLoopVarParamToMethod(Method method, Variable loopVar, EMAMBluePrintCPP bluePrint) {
        String func = method.getInstructions().get(2).getTargetLanguageInstruction();
        if (func.contains(CALC_SUM_METHOD_NAME)) {
            String[] split1 = func.split(CALC_SUM_METHOD_NAME);
            String[] split2 = split1[1].split("[)]");
            func = CALC_SUM_METHOD_NAME + split2[0] + ", " + loopVar.getNameTargetLanguageFormat() + ")";
            // and change the method signiture of the calc sum function
            String mName = CALC_SUM_METHOD_NAME + split1[1].substring(0, split1[1].indexOf("("));
            Optional<Method> affectedMethod = bluePrint.getMethod(mName);
            if (affectedMethod.isPresent()) {
                affectedMethod.get().addParameterUnique(loopVar);
                addLoopVarParamToMethod(affectedMethod.get(), loopVar, bluePrint);
            }
            method.getInstructions().set(2, forLoopBody(func));
        }
    }
}
