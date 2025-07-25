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

public class InRangeCommand extends ArgumentNoReturnMathCommand{
    public InRangeCommand() {
        setMathCommandName("inRange");
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

        Method inRangeHelperMethod = getInRangeHelperMethod(mathMatrixNameExpressionSymbol, bluePrintCPP, properties);
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression("inRangeHelper" + valueListString,mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));

        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        bluePrintCPP.addAdditionalStandardIncludeStringWithHPP("opencv2/core/core");
        bluePrintCPP.addAdditionalUserIncludeStrings("ConvHelper");
        bluePrint.addMethod(inRangeHelperMethod);
        redefineArmaMat(bluePrintCPP);
        redefineInit(bluePrintCPP);
        bluePrintCPP.getGenerator().getCmakeConfig()
                .addModuleDependency(new CMakeFindModule("OpenCV", true).asFindAsPackage());
        bluePrintCPP.addAdditionalNameSpaceStrings("std");

        ConversionHelper.setUsedCV();
    }

    private Method getInRangeHelperMethod(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, EMAMBluePrintCPP bluePrintCPP, MathExpressionProperties properties){
        Method method = new Method("inRangeHelper", "void");

        String typeNameIn = "";
        String typeNameOut = "";


        if(properties.isPreCV()){
            typeNameIn = "cv::Mat";
        } else {
            typeNameIn = "Cube<unsigned char>";
        }

        if(properties.isSucCV()){
            typeNameOut = "cv::Mat";
        }else {
            typeNameOut = "arma::Mat<unsigned char>";
        }

        String typeNameInConst = "const " + typeNameIn +"&";
        String typeNameOutRef = typeNameOut + "&";

        //add parameters
        Variable src = new Variable();
        method.addParameter(src, "src", "CommonMatrix", typeNameInConst, "");;
        Variable dst = new Variable();
        method.addParameter(dst, "dst", "CommonMatrixType", typeNameOutRef, "");
        Variable lowerBoundary = new Variable();
        method.addParameter(lowerBoundary, "lowerB", "colvec", "colvec", "" );
        Variable upperBoundary = new Variable();
        method.addParameter(upperBoundary, "upperB", "colvec", "colvec", "");
        //add an instruction to the method
        method.addInstruction(methodBody(properties, typeNameIn, typeNameOut));

        return method;
    }

    private Instruction methodBody(MathExpressionProperties properties, String typeNameIn, String typeNameOut){
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {

                String finalInstruction ="";

                if (properties.isPreCV() && properties.isSucCV()) {
                    finalInstruction += "    cv::inRange(src, cv::Scalar(lowerB(0), lowerB(1), lowerB(2)),\n" +
                                        "            cv::Scalar(upperB(0), upperB(1), upperB(2)), dst);\n";
                } else if (properties.isPreCV()) {
                    finalInstruction += "    cv::Mat dstCV;\n" +
                                        "    cv::inRange(src, cv::Scalar(lowerB(0), lowerB(1), lowerB(2)),\n" +
                                        "            cv::Scalar(upperB(0), upperB(1), upperB(2)), dstCV);\n" +
                                        "    dst = to_arma<unsigned char>(dstCV);\n";
                } else if (properties.isSucCV()) {
                    finalInstruction += "    cv::Mat srcCV;\n" +
                                        "    srcCV = to_cvmat<unsigned char>(src);\n" +
                                        "    cv::inRange(srcCV, cv::Scalar(lowerB(0), lowerB(1), lowerB(2)),\n" +
                                        "            cv::Scalar(upperB(0), upperB(1), upperB(2)), dst);\n";
                } else {
                    finalInstruction += "    cv::Mat srcCV;\n" +
                                        "    cv::Mat dstCV;\n" +
                                        "    srcCV = to_cvmat<unsigned char>(src);\n" +
                                        "    cv::inRange(srcCV, cv::Scalar(lowerB(0), lowerB(1), lowerB(2)),\n" +
                                        "            cv::Scalar(upperB(0), upperB(1), upperB(2)), dstCV);\n" +
                                        "    dst = to_arma<unsigned char>(dstCV);\n";
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
