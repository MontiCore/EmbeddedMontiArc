package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.MathExpressionProperties;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
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

public class ErodeCommand extends ArgumentNoReturnMathCommand{
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

    @Override
    public boolean isCVMathCommand(){
        return true;
    }

    public void convertUsingOctaveBackend(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint) {
        Log.error("No implementation for Octave Backend");
    }

    public void convertUsingArmadilloBackend(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint) {
        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;
        mathMatrixNameExpressionSymbol.setNameToAccess("");
        MathExpressionProperties properties = ComponentConverter.tuples.get(mathExpressionSymbol);

        BluePrintCPP bluePrintCPP  = (BluePrintCPP) bluePrint;
        String valueListString = "";
        List<MathMatrixAccessSymbol> mathMatrixAccessSymbols= mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols();
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixAccessSymbols)
            MathFunctionFixer.fixMathFunctions(accessSymbol, bluePrintCPP);

        Method erodeHelperMethod = getErodeHelperMethod(mathMatrixNameExpressionSymbol, bluePrintCPP, properties);
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression("erodeHelper" + valueListString,mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));


        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        bluePrintCPP.addCVIncludeString("opencv2/imgproc/imgproc");
        bluePrintCPP.addCVIncludeString("ConvHelper");
        bluePrint.addMethod(erodeHelperMethod);
        redefineArmaMat(bluePrintCPP);
        redefineInit(bluePrintCPP);

    }

    private Method getErodeHelperMethod(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, BluePrintCPP bluePrintCPP, MathExpressionProperties properties){
        Method method = new Method("erodeHelper", "void");

        String typeName = getTypeOfFirstInput(mathMatrixNameExpressionSymbol, bluePrintCPP);
        String typeNameIn = "";
        String typeNameOut = "";

        if(typeName.equals("") || typeName.equals("mat")){
            typeName = "arma::Mat<unsigned char>";
        }else if(typeName.equals("cube")){
            typeName = "Cube<unsigned char>";
        }

        if(properties.isPreCV()){
            typeNameIn = "cv::Mat";
        } else {
            typeNameIn = typeName;
        }

        if(properties.isSucCV()){
            typeNameOut = "cv::Mat";
        }else {
            typeNameOut = typeName;
        }
        String typeNameInConst = "const " + typeNameIn +"&";
        String typeNameOutRef = typeNameOut + "&";

        //add parameters
        Variable src = new Variable();
        method.addParameter(src, "src", "CommonMatrixType", typeNameInConst, MathConverter.curBackend.getIncludeHeaderName());
        Variable dst = new Variable();
        method.addParameter(dst, "dst", "CommonMatrixType", typeNameOutRef, MathConverter.curBackend.getIncludeHeaderName());
        Variable erosion_elem = new Variable();
        method.addParameter(erosion_elem,"erosion_elem", "Integer", "int", "");
        Variable iterations = new Variable();
        method.addParameter(iterations, "iterations", "Integer","int", "");
        //add an instruction to the method
        method.addInstruction(methodBody(properties, typeNameIn, typeNameOut));

        return method;
    }

    private Instruction methodBody(MathExpressionProperties properties, String typeNameIn, String typeNameOut) {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                String finalInstruction =   "    int erosion_type = 0;\n" +
                                            "    if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }\n" +
                                            "    else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }\n" +
                                            "    else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }\n" +
                                            "    int erosion_size = erosion_elem;\n" +
                                            "    cv::Mat element = cv::getStructuringElement( erosion_type,\n" +
                                            "                            cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),\n" +
                                            "                            cv::Point( -1, -1 ) );\n";

                if(properties.isPreCV() && properties.isSucCV()){
                    finalInstruction += "    cv::erode( src, dst, element, cv::Point(-1,-1), iterations );\n";
                }else if(properties.isPreCV()){
                    finalInstruction += "    cv::Mat dstCV;\n" +
                                        "    cv::erode( src, dstCV, element, cv::Point(-1,-1), iterations );\n";
                    if(typeNameOut == "cube"){
                        finalInstruction += "    dst = to_armaCube<unsigned char, 3>(dstCV);\n";
                    }   else {
                        finalInstruction += "    dst = to_arma<unsigned char>(dstCV);\n";
                    }
                } else if(properties.isSucCV()){
                    finalInstruction += "    cv::Mat srcCV;\n" +
                                        "    srcCV = to_cvmat<unsigned char>(src);\n" +
                                        "    cv::erode( srcCV, dst, element, cv::Point(-1,-1), iterations );\n";
                } else {
                    finalInstruction += "    cv::Mat srcCV;\n" +
                                        "    cv::Mat dstCV;\n" +
                                        "    srcCV = to_cvmat<unsigned char>(src);\n" +
                                        "    cv::erode( srcCV, dstCV, element, cv::Point(-1,-1), iterations );\n";
                    if(typeNameOut == "cube"){
                        finalInstruction += "    dst = to_armaCube<unsigned char, 3>(dstCV);\n";
                    }   else {
                        finalInstruction += "    dst = to_arma<unsigned char>(dstCV);\n";
                    }
                }
                return  finalInstruction;
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }
}