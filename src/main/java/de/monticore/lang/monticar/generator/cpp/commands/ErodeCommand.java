package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.cpp.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Ahmed Diab
 */

public class ErodeCommand extends ArgumentReturnMathCommand{
    public ErodeCommand() {
        setMathCommandName("erode");
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

        Method erodeHelperMethod = getErodeHelperMethod();
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression("erodeHelper" + valueListString,mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));


        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        ((BluePrintCPP) bluePrint).addCVIncludeString("opencv2/imgproc");
        bluePrint.addMethod(erodeHelperMethod);

    }

    private Method getErodeHelperMethod(){
        Method method = new Method("erodeHelper", "void");

        //parameter
        Variable src = new Variable();
        src.setName("src");
        src.setVariableType(new VariableType("CommonMatrixType", MathConverter.curBackend.getMatrixTypeName(), MathConverter.curBackend.getIncludeHeaderName()));
        Variable dst = new Variable();
        dst.setName("dst");
        dst.setVariableType(new VariableType("CommonMatrixType", MathConverter.curBackend.getMatrixTypeName(), MathConverter.curBackend.getIncludeHeaderName()));
        Variable erosion_elem = new Variable();
        erosion_elem.setName("erosion_elem");
        erosion_elem.setVariableType(new VariableType("Integer", "int",""));
        Variable iterations = new Variable();
        iterations.setName("iterations");
        iterations.setVariableType(new VariableType("Integer", "int", ""));
        //add variable to method
        method.addParameter(src);
        method.addParameter(dst);
        method.addParameter(erosion_elem);
        method.addParameter(iterations);
        //add an instruction to the method
        method.addInstruction(methodBody());

        return method;
    }

    private Instruction methodBody() {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                return  "    int erosion_type = 0;\n" +
                        "    if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }\n" +
                        "    else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }\n" +
                        "    else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }\n" +
                        "    erosion_size = erosion_elem;\n" +
                        "    mat element = getStructuringElement( erosion_type,\n" +
                        "                            Size( 2*erosion_size + 1, 2*erosion_size+1 ),\n" +
                        "                            Point( erosion_size, erosion_size ) );\n" +
                        "    erode( src, dst, element, Point(-1,-1), iterations );";
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }
}
