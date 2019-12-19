package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueType;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.OctaveHelper;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.cpp.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MathScaleCubeCommand extends MathCommand {
    //todo
    private static final String SCALER_SYNTAX_EXTENDED = "scaleCube( EXPRESSION , AXIS , NEW_X , NEW_Y )";

    private static final String SCALER_METHOD_NAME = "scaleCube";

    private static int scalerCommandCounter = 0;

    public MathScaleCubeCommand() {
        setMathCommandName("scaleCube");
    }

    @Override
    public void convert(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint) {
        String backendName = MathConverter.curBackend.getBackendName();
        if (backendName.equals("OctaveBackend")) {
            convertUsingOctaveBackend(mathExpressionSymbol, bluePrint);
        } else if (backendName.equals("ArmadilloBackend")) {
            convertUsingArmadilloBackend(mathExpressionSymbol, bluePrint);
        }

    }

    public void convertUsingOctaveBackend(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint) {
        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;

        mathMatrixNameExpressionSymbol.setNameToAccess("");

        String valueListString = "";
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
            MathFunctionFixer.fixMathFunctions(accessSymbol, (BluePrintCPP) bluePrint);
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<>());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression(OctaveHelper.getCallBuiltInFunction(mathExpressionSymbol, "Fsum", "Double", valueListString, "FirstResult", false, 1), mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));

        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        ((BluePrintCPP) bluePrint).addAdditionalIncludeString("octave/builtin-defun-decls");
        // error if using extended syntax here
        if (mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().size() == 4) {
            //todo
            Log.error(String.format("Syntax: \"%s\" is not supported when using deprecated backend Octave", SUM_SYNTAX_EXTENDED));
        }
    }

    public void convertUsingArmadilloBackend(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint) {
        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;
        mathMatrixNameExpressionSymbol.setNameToAccess("");

        String valueListString = "";
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
            MathFunctionFixer.fixMathFunctions(accessSymbol, (BluePrintCPP) bluePrint);
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());

        BluePrintCPP bluePrintCPP = (BluePrintCPP) bluePrint;
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
            MathFunctionFixer.fixMathFunctions(accessSymbol, bluePrintCPP);
        if (mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().size() == 4) {
            MathMatrixAccessSymbol cube = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(0);
            MathMatrixAccessSymbol axis = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(1);
            MathMatrixAccessSymbol new_x = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(2);
            MathMatrixAccessSymbol new_y = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(3);
            convertExtendedScalerImplementationArmadillo(valueListString, mathMatrixNameExpressionSymbol, cube, axis, new_x, new_y, bluePrintCPP);
        } else {
            //todo
            Log.error(String.format("No implementation found for sum operation: \"sum(%s)\". Possible syntax is \"sum( X )\", \"sum(X,dim)\" or \"%s\"", mathExpressionSymbol.getTextualRepresentation(), SUM_SYNTAX_EXTENDED));
        }
    }

    /**
     * Implements a sum function with syntax "sum( EXPRESSION , SUM_VARIABLE , START_VALUE , END_VALUE )"
     * This syntax makes sum expressions easier to model.
     *
     * @param mathMatrixNameExpressionSymbol symbol to convert
     * @param cube                           expression from which the sum is calculates
     * @param axis                           name of the sum variable
     * @param new_x                          start value of the sum variable
     * @param new_y                          end value of the sum variable
     */
    private void convertExtendedScalerImplementationArmadillo(String valueString, MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, MathMatrixAccessSymbol cube, MathMatrixAccessSymbol axis, MathMatrixAccessSymbol new_x, MathMatrixAccessSymbol new_y, BluePrintCPP bluePrint) {
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setAccessStartSymbol("");
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setAccessEndSymbol("");
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().clear();
        // create method
        Method calcSumMethod = getScalerCalculationMethod(cube, axis, new_x, new_y, bluePrint);
        // create code string
        String code = calcSumMethod.getName() + valueString;
        MathStringExpression codeExpr = new MathStringExpression(code, mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().add(new MathMatrixAccessSymbol(codeExpr));
        // add method to bluePrint
        bluePrint.addMethod(calcSumMethod);
    }

    private Method getScalerCalculationMethod(MathMatrixAccessSymbol cube, MathMatrixAccessSymbol axis, MathMatrixAccessSymbol new_x2, MathMatrixAccessSymbol new_y2, BluePrint bluePrint) {
        // create new method
        Method method = getNewEmptyScalerCalculationMethod();

        // generate function code
        String c = ExecuteMethodGenerator.generateExecuteCode(cube, new ArrayList<>());
        String a = ExecuteMethodGenerator.generateExecuteCode(axis, new ArrayList<>());
        String n_x = ExecuteMethodGenerator.generateExecuteCode(new_x2, new ArrayList<>());
        String n_y = ExecuteMethodGenerator.generateExecuteCode(new_y2, new ArrayList<>());
        // add loop var
        // parameters
        Variable img = new Variable();
        img.setName("img");
        img.setVariableType(new VariableType("Cube", "cube", ""));
        Variable new_x = new Variable();
        new_x.setName("new_x");
        new_x.setVariableType(new VariableType("Integer", "int", ""));
        Variable new_y = new Variable();
        new_y.setName("new_y");
        new_y.setVariableType(new VariableType("Integer", "int", ""));
        Variable depth_axis = new Variable();
        depth_axis.setName("depth_axis");
        depth_axis.setVariableType(new VariableType("Integer", "int", ""));
        method.addParameter(img);
        method.addParameter(new_x);
        method.addParameter(new_y);
        method.addParameter(depth_axis);
        // add instructions


        method.addInstruction(ifClauses());
        return method;
    }

    private Method getNewEmptyScalerCalculationMethod() {
        scalerCommandCounter++;
        Method method = new Method();
        method.setName(SCALER_METHOD_NAME + scalerCommandCounter);
        method.setReturnTypeName("cube");
        return method;
    }

    private Instruction ifClauses() {
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
                        "        arma::vec X = arma::regspace(1, cur_slice.n_cols);\n" +
                        "        arma::vec Y = arma::regspace(1, cur_slice.n_rows);\n" +
                        "\n" +
                        "        float scale_x = cur_slice.n_cols/new_x;\n" +
                        "        float scale_y = cur_slice.n_rows/new_y;\n" +
                        "        arma::vec XI = arma::regspace(1, new_x);\n" +
                        "        arma::vec YI = arma::regspace(1, new_y);\n" +
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
                        "    return r_img;";
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }
}
