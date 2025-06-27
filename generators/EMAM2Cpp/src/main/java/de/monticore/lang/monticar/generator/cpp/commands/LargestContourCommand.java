package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.cpp.ConversionHelper;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Ahmed Diab
 */

public class LargestContourCommand extends MathCommand {
    public LargestContourCommand() {
        setMathCommandName("largestContour");
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
        EMAMBluePrintCPP bluePrintCPP = (EMAMBluePrintCPP) bluePrint;
        String valueListString = "";
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
            MathFunctionFixer.fixMathFunctions(accessSymbol, bluePrintCPP);

        String nameOfFirstParameter = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(0).getTextualRepresentation();
        ComponentConverter.fixVariableType(nameOfFirstParameter, bluePrintCPP, "Q", "std::vector<std::vector<cv::Point>>", "");
        Method largestContourMethod = getLargestContourMethod();
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression("largestContour" + valueListString,mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));


        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        bluePrintCPP.addAdditionalStandardIncludeStringWithHPP("opencv2/imgproc/imgproc");
        bluePrint.addMethod(largestContourMethod);
        redefineArmaMat(bluePrintCPP);
        redefineInit(bluePrintCPP);
        bluePrintCPP.getGenerator().getCmakeConfig()
                .addModuleDependency(new CMakeFindModule("OpenCV", true).asFindAsPackage());
        bluePrintCPP.addAdditionalNameSpaceStrings("std");

        ConversionHelper.setUsedCV();
    }

    private Method getLargestContourMethod(){
        Method method = new Method("largestContour", "std::vector<cv::Point>");

        //add parameters
        Variable contours = new Variable();
        method.addParameter(contours, "contours", "double","const std::vector<std::vector<cv::Point>>&", "");
        //add an instruction to the method
        method.addInstruction(methodBody());

        return method;
    }

    private Instruction methodBody() {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                return  "    std::vector<cv::Point> singleContour;\n" +
                        "    double maxArea = 0;\n" +
                        "    int maxAreaContourId = 0;\n" +
                        "   for (int j = 0; j < contours.size(); j++) {\n" +
                        "       double newArea = cv::contourArea(contours.at(j));\n" +
                        "       if (newArea > maxArea) {\n" +
                        "           maxArea = newArea;\n" +
                        "           maxAreaContourId = j;\n" +
                        "       }\n" +
                        "   }\n" +
                        "   return contours.empty()? singleContour: contours.at(maxAreaContourId);\n";
            }

            @Override
            public boolean isConnectInstruction() {
                return false;
            }
        };
    }


}
