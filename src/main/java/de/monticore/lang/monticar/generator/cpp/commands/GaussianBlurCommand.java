package de.monticore.lang.monticar.generator.cpp.commands;

import alice.tuprolog.Int;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.cpp.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Ahmed Diab
 */

public class GaussianBlurCommand extends ArgumentReturnMathCommand{
    public GaussianBlurCommand() {
        setMathCommandName("gaussianBlur");
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
        Log.error("No implementation for Octave Backend");
    }

    public void convertUsingArmadilloBackend(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint) {
        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;
        mathMatrixNameExpressionSymbol.setNameToAccess("");

        String valueListString = "";
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
            MathFunctionFixer.fixMathFunctions(accessSymbol, (BluePrintCPP) bluePrint);

        Method gaussianBlurHelperMethod = getGaussianBlurHelperMethod();
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression("gaussianBlurHelper" + valueListString,mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));

        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        ((BluePrintCPP) bluePrint).addCVIncludeString("opencv2/imgproc");
        bluePrint.addMethod(gaussianBlurHelperMethod);

    }

    private Method getGaussianBlurHelperMethod(){
        Method method = new Method("gaussianBlurHelper", "void");

        //parameters
        Variable src = new Variable();
        src.setName("src");
        src.setVariableType(new VariableType("CommonMatrixType", MathConverter.curBackend.getMatrixTypeName(), MathConverter.curBackend.getIncludeHeaderName()));
        Variable dst = new Variable();
        dst.setName("dst");
        dst.setVariableType(new VariableType("CommonMatrixType", MathConverter.curBackend.getMatrixTypeName(), MathConverter.curBackend.getIncludeHeaderName()));
        Variable sizeX = new Variable();
        sizeX.setName("sizeX");
        sizeX.setVariableType(new VariableType("Integer", "int",""));
        Variable sizeY = new Variable();
        sizeY.setName("sizeY");
        sizeY.setVariableType(new VariableType("Integer", "int",""));
        Variable sigmaX = new Variable();
        sigmaX.setName("sigmaX");
        sigmaX.setVariableType(new VariableType("Double", "double",""));
        Variable sigmaY = new Variable();
        sigmaY.setName("sigmaY");
        sigmaY.setVariableType(new VariableType("Double", "double",""));
        method.addParameter(src);
        method.addParameter(dst);
        method.addParameter(sizeX);
        method.addParameter(sizeY);
        method.addParameter(sigmaX);
        method.addParameter(sigmaY);

        method.addInstruction(methodBody());

        return method;

    }

    private Instruction methodBody(){
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                return "    gaussianBlur(src, dst, Size(sizeX, sizeY), sigmaX, sigmaY);\n";
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }
}
