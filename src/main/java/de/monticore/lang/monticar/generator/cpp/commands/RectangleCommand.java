package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.cpp.ConversionHelper;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.MathExpressionProperties;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Ahmed Diab
 */

public class RectangleCommand extends MathCommand{
    public RectangleCommand() {
        setMathCommandName("rectangle");
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

    @Override
    public boolean isCVMathCommand(){
        return true;
    }

    public void convertUsingOctaveBackend(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrint bluePrint) {
        Log.error("No implementation for Octave Backend");
    }

    public void convertUsingArmadilloBackend(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrint bluePrint) {
        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;
        mathMatrixNameExpressionSymbol.setNameToAccess("");
        MathExpressionProperties properties = ComponentConverter.tuples.get(mathExpressionSymbol);

        EMAMBluePrintCPP bluePrintCPP = (EMAMBluePrintCPP) bluePrint;
        String valueListString = "";
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
            MathFunctionFixer.fixMathFunctions(accessSymbol, bluePrintCPP);

        String nameOfSecondParameter = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(1).getTextualRepresentation();
        ComponentConverter.fixVariableType(nameOfSecondParameter, bluePrintCPP, "Q", "cv::Rect", "");

        Method rectangleHelperMethod = getRectangleHelperMethod(mathMatrixNameExpressionSymbol, bluePrintCPP, properties);
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression("rectangleHelper" + valueListString,mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));


        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        bluePrintCPP.addAdditionalStandardIncludeStringWithHPP("opencv2/imgproc/imgproc");
        bluePrintCPP.addAdditionalUserIncludeStrings("ConvHelper");
        bluePrint.addMethod(rectangleHelperMethod);
        redefineArmaMat(bluePrintCPP);
        redefineInit(bluePrintCPP);
        bluePrintCPP.getGenerator().getCmakeConfig()
                .addModuleDependency(new CMakeFindModule("OpenCV", true).asFindAsPackage());
        bluePrintCPP.addAdditionalNameSpaceStrings("std");

        ConversionHelper.setUsedCV();
    }

    private Method getRectangleHelperMethod(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, EMAMBluePrintCPP bluePrintCPP, MathExpressionProperties properties){
        Method method = new Method("rectangleHelper", "arma::mat");
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
            method.setReturnTypeName(typeNameOut);

        }else {
            typeNameOut = typeName;
            method.setReturnTypeName(typeNameOut);
        }

        String typeNameInConst = "const " + typeNameIn +"&";

        //add parameters
        Variable src = new Variable();
        method.addParameter(src, "src", "CommonMatrixType",typeNameInConst, "");
        Variable rect = new Variable();
        method.addParameter(rect, "rect", "double", "cv::Rect&", "");
        Variable color = new Variable();
        method.addParameter(color,"color", "colvec", "colvec", "");
        Variable thickness = new Variable();
        method.addParameter(thickness, "thickness", "Integer","int", "");
        Variable lineType = new Variable();
        method.addParameter(lineType, "lineType", "Integer","int", "");
        //add an instruction to the method
        method.addInstruction(methodBody(properties, typeNameIn, typeNameOut));

        return method;
    }

    private Instruction methodBody(MathExpressionProperties properties, String typeNameIn, String typeNameOut) {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                String finalInstruction = "";

                if(properties.isPreCV() && properties.isSucCV()){
                    finalInstruction =  "    cv::rectangle(src, rect.tl(), rect.br(), cv::Scalar(color(0), color(1), color(2)), thickness, lineType);\n" +
                                        "    return src;\n";
                } else if(properties.isPreCV()){
                    finalInstruction =  "    cv::rectangle(src, rect.tl(), rect.br(), cv::Scalar(color(0), color(1), color(2)), thickness, lineType);\n";
                    if(typeNameOut == "Cube<unsigned char>"){
                        finalInstruction += "    arma::Cube<unsigned char> srcCube;\n" +
                                            "    srcCube = to_armaCube<unsigned char, 3>(src);\n" +
                                            "    return srcCube;\n";
                    } else {
                        finalInstruction += "    arma::mat srcArma;\n" +
                                            "    srcArma = to_arma<unsigned char>(src);\n" +
                                            "    return srcArma;\n";
                    }
                } else if(properties.isSucCV()){
                    finalInstruction =  "    cv::Mat srcCV;\n" +
                                        "    srcCV = to_cvmat<unsigned char>(src);\n" +
                                        "    cv::rectangle(srcCV, rect.tl(), rect.br(), cv::Scalar(color(0), color(1), color(2)), thickness, lineType);\n" +
                                        "    return srcCV;\n";
                } else {
                    finalInstruction =  "    cv::Mat srcCV;\n" +
                                        "    srcCV = to_cvmat<unsigned char>(src);\n" +
                                        "    cv::rectangle(srcCV, rect.tl(), rect.br(), cv::Scalar(color(0), color(1), color(2)), thickness, lineType);\n";
                    if(typeNameOut == "Cube<unsigned char>"){
                        finalInstruction += "    arma::Cube<unsigned char> srcCube;\n" +
                                            "    srcCube = to_armaCube<unsigned char, 3>(srcCV);\n" +
                                            "    return srcCube;\n";
                    }   else {
                        finalInstruction += "    arma::mat srcArma;\n" +
                                            "    srcArma = to_arma<unsigned char>(srcCV);\n" +
                                            "    return srcArma;\n";
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
