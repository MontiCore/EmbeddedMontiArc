package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class MathJoinCubeDimCommand extends MathCommand {
    //todo
    private static final String JOINER_SYNTAX_EXTENDED = "joinCubeDim(  CUBE , CUBE , DIM )";

    private static final String JOINER_METHOD_NAME = "joinCubeDim";

    private static int scalerCommandCounter = 0;

    public MathJoinCubeDimCommand() {
        setMathCommandName("joinCubeDim");
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
        Log.error("Not implemented for Octave Backend");
    }

    public void convertUsingArmadilloBackend(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrint bluePrint) {
        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;
        mathMatrixNameExpressionSymbol.setNameToAccess("");

        String valueListString = "";
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
            MathFunctionFixer.fixMathFunctions(accessSymbol, (EMAMBluePrintCPP) bluePrint);
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());

        EMAMBluePrintCPP bluePrintCPP = (EMAMBluePrintCPP) bluePrint;
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
            MathFunctionFixer.fixMathFunctions(accessSymbol, bluePrintCPP);
        if (mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().size() == 3) {
            MathMatrixAccessSymbol cube1 = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(0);
            MathMatrixAccessSymbol cube2 = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(1);
            MathMatrixAccessSymbol axis = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(2);
            convertExtendedJoinerImplementationArmadillo(valueListString, mathMatrixNameExpressionSymbol, bluePrintCPP);
        } else {
            //todo
            Log.error(String.format("No implementation found for joinCubeDim operation: \"joinCubeDim(%s)\".", mathExpressionSymbol.getTextualRepresentation(), JOINER_SYNTAX_EXTENDED));
        }
    }

    /**
     * Implements a scaleCube function with syntax "scaleCube( CUBE , AXIS , NEW_X , NEW_Y )"
     *
     */
    private void convertExtendedJoinerImplementationArmadillo(String valueString, MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, EMAMBluePrint bluePrint) {
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setAccessStartSymbol("");
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setAccessEndSymbol("");
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().clear();
        // create method
        Method calcJoinerMethod = getJoinerCalculationMethod(bluePrint);
        // create code string
        String code = calcJoinerMethod.getName() + valueString;
        MathStringExpression codeExpr = new MathStringExpression(code, mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().add(new MathMatrixAccessSymbol(codeExpr));
        // add method to bluePrint
        bluePrint.addMethod(calcJoinerMethod);
    }

    private Method getJoinerCalculationMethod(EMAMBluePrint bluePrint) {
        // create new method
        Method method = getNewEmptyScalerCalculationMethod();

        // parameters
        Variable c1 = new Variable();
        c1.setName("c1");
        c1.setVariableType(new VariableType("Cube", "cube", ""));
        Variable c2 = new Variable();
        c2.setName("c2");
        c2.setVariableType(new VariableType("Cube", "cube", ""));
        Variable dim = new Variable();
        dim.setName("dim");
        dim.setVariableType(new VariableType("Integer", "int", ""));

        method.addParameter(c1);
        method.addParameter(c2);
        method.addParameter(dim);

        // add instructions
        method.addInstruction(methodBody());

        return method;
    }

    private Method getNewEmptyScalerCalculationMethod() {
        scalerCommandCounter++;
        Method method = new Method();
        method.setName(JOINER_METHOD_NAME + scalerCommandCounter);
        method.setReturnTypeName("cube");
        return method;
    }

    private Instruction methodBody() {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                return "    if (dim == 0) {\n" +
                        "        c1.insert_rows(c1.n_rows, c2);\n" +
                        "        return c1;\n" +
                        "    } else if(dim == 1) {\n" +
                        "        c1.insert_cols(c1.n_cols, c2);\n" +
                        "        return c1;\n" +
                        "    }\n" +
                        "\n" +
                        "    c1 = arma::join_slices(c1, c2);\n" +
                        "    return c1;";
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }
}
