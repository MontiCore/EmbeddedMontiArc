package de.monticore.lang.monticar.generator.cpp.commands;

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

public class DilateCommand extends ArgumentReturnMathCommand{
    public DilateCommand() {
        setMathCommandName("dilate");
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

        Method dilateHelperMethod = getDilateHelperMethod();
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression("dilateHelper" + valueListString,mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));

        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        ((BluePrintCPP) bluePrint).addCVIncludeString("opencv2/imgproc");
        bluePrint.addMethod(dilateHelperMethod);

    }

    private Method getDilateHelperMethod(){
        Method method = new Method("dilateHelper", "void");

        //add parameters
        Variable src = new Variable();
        method.addParameter(src, "src", "CommonMatrix",MathConverter.curBackend.getMatrixTypeName(), MathConverter.curBackend.getIncludeHeaderName());;
        Variable dst = new Variable();
        method.addParameter(dst, "dst", "CommonMatrixType", MathConverter.curBackend.getMatrixTypeName(), MathConverter.curBackend.getIncludeHeaderName());
        Variable erosion_elem = new Variable();
        method.addParameter(erosion_elem,"dilation_elem", "Integer", "int", "");
        Variable iterations = new Variable();
        method.addParameter(iterations, "iterations", "Integer","int", "");
        //add an instruction to the method
        method.addInstruction(methodBody());


        return method;
    }

    private Instruction methodBody() {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                return  "    int dilation_type = 0;\n" +
                        "    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }\n" +
                        "    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }\n" +
                        "    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }\n" +
                        "    dilation_size = dilation_elem;\n" +
                        "    mat element = getStructuringElement( dilation_type,\n" +
                        "                            Size( 2*dilation_size + 1, 2*dilation_size+1 ),\n" +
                        "                            Point( -1, -1 ) );\n" +
                        "    dilate( src, dst, element, Point(-1,-1), iterations );\n";
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }
}
