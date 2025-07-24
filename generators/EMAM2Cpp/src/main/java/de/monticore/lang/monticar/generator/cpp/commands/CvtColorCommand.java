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

public class CvtColorCommand extends ArgumentNoReturnMathCommand{
    public CvtColorCommand() {
        setMathCommandName("cvtColor");
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

        Method cvtColorHelperMethod = getCvtColorHelperMethod(mathMatrixNameExpressionSymbol, bluePrintCPP, properties);
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression("cvtColorHelper" + valueListString,mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));

        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        bluePrintCPP.addAdditionalStandardIncludeStringWithHPP("opencv2/imgproc/imgproc");
        bluePrintCPP.addAdditionalUserIncludeStrings("ConvHelper");
        bluePrint.addMethod(cvtColorHelperMethod);
        redefineArmaMat(bluePrintCPP);
        redefineInit(bluePrintCPP);
        bluePrintCPP.getGenerator().getCmakeConfig()
                .addModuleDependency(new CMakeFindModule("OpenCV", true).asFindAsPackage());
        bluePrintCPP.addAdditionalNameSpaceStrings("std");

        ConversionHelper.setUsedCV();
    }

    private Method getCvtColorHelperMethod(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, EMAMBluePrintCPP bluePrintCPP, MathExpressionProperties properties){
        Method method = new Method("cvtColorHelper", "void");

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
        Variable coloerConversion = new Variable();
        method.addParameter(coloerConversion,"colorConversion", "Integer", "int", "");
        //add an instruction to the method
        method.addInstruction(methodBody(properties, typeNameIn, typeNameOut));

        return method;
    }

    private Instruction methodBody(MathExpressionProperties properties, String typeNameIn, String typeNameOut) {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                String finalInstruction ="";

                if(properties.isPreCV() && properties.isSucCV()){
                    finalInstruction += "    cv::cvtColor(src, dst, colorConversion);\n";
                }else if(properties.isPreCV()){
                    finalInstruction += "    cv::Mat dstCV;\n" +
                                        "    cv::cvtColor(src, dstCV, colorConversion);\n";
                    if(typeNameOut == "Cube<unsigned char>"){
                        finalInstruction += "    dst = to_armaCube<unsigned char, 3>(dstCV);\n";
                    }   else {
                        finalInstruction += "    dst = to_arma<unsigned char>(dstCV);\n";
                    }
                } else if(properties.isSucCV()){
                    finalInstruction += "    cv::Mat srcCV;\n" +
                                        "    srcCV = to_cvmat<unsigned char>(src);\n" +
                                        "    cv::cvtColor(srcCV, dst, colorConversion);\n";
                } else {
                    finalInstruction += "    cv::Mat srcCV;\n" +
                                        "    cv::Mat dstCV;\n" +
                                        "    srcCV = to_cvmat<unsigned char>(src);\n" +
                                        "    cv::cvtColor(srcCV, dstCV, colorConversion);\n";
                    if(typeNameOut == "Cube<unsigned char>"){
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
