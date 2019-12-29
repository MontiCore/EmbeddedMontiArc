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

public class DilateCommand extends MathCommand{
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

        //parameter
        Variable src = new Variable();
        src.setName("src");
        src.setVariableType(new VariableType("CommonMatrixType", MathConverter.curBackend.getMatrixTypeName(), MathConverter.curBackend.getIncludeHeaderName()));
        Variable dst = new Variable();
        dst.setName("dst");
        dst.setVariableType(new VariableType("CommonMatrixType", MathConverter.curBackend.getMatrixTypeName(), MathConverter.curBackend.getIncludeHeaderName()));
        Variable dilation_elem = new Variable();
        dilation_elem.setName("erosion_elem");
        dilation_elem.setVariableType(new VariableType("Integer", "int",""));
        Variable iteration = new Variable();
        iteration.setName("iteration");
        iteration.setVariableType(new VariableType("Integer", "int", ""));
        //add variable to method
        method.addParameter(src);
        method.addParameter(dst);
        method.addParameter(dilation_elem);
        method.addParameter(iteration);
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
                        "                            Point( dilation_size, dilation_size ) );\n" +
                        "    dilate( src, dst, element, Point(-1,-1), iteration );";
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }
}
