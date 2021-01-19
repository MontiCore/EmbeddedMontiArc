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

public class MathScaleCubeCommand extends MathCommand {
    private static final String SCALER_SYNTAX_EXTENDED = "scaleCube( EXPRESSION , AXIS , NEW_X , NEW_Y )";

    private static final String SCALER_METHOD_NAME = "scaleCube";

    private static int scalerCommandCounter = 0;

    public MathScaleCubeCommand() {
        setMathCommandName("scaleCube");
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
        if (mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().size() == 4) {
            MathMatrixAccessSymbol cube = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(0);
            MathMatrixAccessSymbol axis = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(1);
            MathMatrixAccessSymbol new_x = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(2);
            MathMatrixAccessSymbol new_y = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(3);
            convertExtendedScalerImplementationArmadillo(valueListString, mathMatrixNameExpressionSymbol, bluePrintCPP);
        } else {
            Log.error(String.format("No implementation found for scaleCube operation: \"scaleCube(%s)\".", mathExpressionSymbol.getTextualRepresentation(), SCALER_SYNTAX_EXTENDED));
        }
    }

    /**
     * Implements a scaleCube function with syntax "scaleCube( CUBE , AXIS , NEW_X , NEW_Y )"
     *
     */
    private void convertExtendedScalerImplementationArmadillo(String valueString, MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, EMAMBluePrint bluePrint) {
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setAccessStartSymbol("");
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setAccessEndSymbol("");
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().clear();
        // create method
        Method calcScalerMethod = getScalerCalculationMethod(bluePrint);
        // create code string
        String code = calcScalerMethod.getName() + valueString;
        MathStringExpression codeExpr = new MathStringExpression(code, mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().add(new MathMatrixAccessSymbol(codeExpr));
        // add method to bluePrint
        bluePrint.addMethod(calcScalerMethod);
    }

    private Method getScalerCalculationMethod(EMAMBluePrint bluePrint) {
        // create new method
        Method method = getNewEmptyScalerCalculationMethod();

        // parameters
        Variable img = new Variable();
        img.setName("img");
        img.setVariableType(new VariableType("Cube", "cube", ""));
        Variable depth_axis = new Variable();
        depth_axis.setName("depth_axis");
        depth_axis.setVariableType(new VariableType("Integer", "int", ""));
        Variable new_x = new Variable();
        new_x.setName("new_x");
        new_x.setVariableType(new VariableType("Integer", "int", ""));
        Variable new_y = new Variable();
        new_y.setName("new_y");
        new_y.setVariableType(new VariableType("Integer", "int", ""));
        method.addParameter(img);
        method.addParameter(depth_axis);
        method.addParameter(new_x);
        method.addParameter(new_y);

        // add instructions
        method.addInstruction(methodBody());

        return method;
    }

    private Method getNewEmptyScalerCalculationMethod() {
        scalerCommandCounter++;
        Method method = new Method();
        method.setName(SCALER_METHOD_NAME + scalerCommandCounter);
        method.setReturnTypeName("cube");
        return method;
    }

    private Instruction methodBody() {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                return  "    if (depth_axis == 0) { \n "+
                        "       img = arma::reshape(img, img.n_cols, img.n_slices, img.n_rows);\n" +
                        "    } else if(depth_axis == 1) {\n" +
                        "        img = arma::reshape(img, img.n_rows, img.n_slices, img.n_cols);\n" +
                        "    }\n" +
                        "    \n" +
                        "    arma::cube r_img = arma::cube(64,64, img.n_slices);\n" +
                        "    for (int i = 0; i < img.n_slices; i++) \n" +
                        "    {\n" +
                        "        arma::mat cur_slice = img.slice(i);\n" +
                        "        arma::vec X = arma::regspace(0, cur_slice.n_cols-1);\n" +
                        "        arma::vec Y = arma::regspace(0, cur_slice.n_rows-1);\n" +
                        "\n" +
                        "        float scale_x = (cur_slice.n_cols-1)/float((new_x));\n" +
                        "        float scale_y = (cur_slice.n_rows-1)/float((new_y));\n" +
                        "        arma::vec XI = arma::regspace(0, new_x-1) * scale_x;\n" +
                        "        arma::vec YI = arma::regspace(0, new_y-1) * scale_y;\n" +
                        "\n" +
                        "        arma::mat mat_out;\n" +
                        "\n" +
                        "        arma::interp2(X, Y, cur_slice, XI, YI, mat_out);\n" +
                        "        r_img.slice(i) = mat_out;\n" +
                        "    }\n" +
                        "\n" +
                        "    if (depth_axis == 0) {\n" +
                        "        r_img = arma::reshape(r_img, r_img.n_slices, r_img.n_rows, r_img.n_cols);\n" +
                        "    } else if (depth_axis == 1) {\n" +
                        "        r_img = arma::reshape(r_img, r_img.n_rows, r_img.n_slices, r_img.n_cols);\n" +
                        "    }\n" +
                        "    \n" +
                        "    return r_img;\n";
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }
}
