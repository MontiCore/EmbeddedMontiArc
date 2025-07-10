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

public class FindContoursCommand extends ArgumentNoReturnMathCommand{
    public FindContoursCommand() {
        setMathCommandName("findContours");
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

        Method findContoursHelperMethod = getFindContoursHelperMethod(mathMatrixNameExpressionSymbol, bluePrintCPP, properties);
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression("findContoursHelper" + valueListString,mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));

        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        bluePrintCPP.addAdditionalStandardIncludeStringWithHPP("opencv2/imgproc/imgproc");
        bluePrintCPP.addAdditionalUserIncludeStrings("ConvHelper");
        bluePrintCPP.addAdditionalStandardInclude("vector");
        bluePrint.addMethod(findContoursHelperMethod);
        redefineArmaMat(bluePrintCPP);
        redefineInit(bluePrintCPP);
        bluePrintCPP.getGenerator().getCmakeConfig()
                .addModuleDependency(new CMakeFindModule("OpenCV", true).asFindAsPackage());
        bluePrintCPP.addAdditionalNameSpaceStrings("std");

        ConversionHelper.setUsedCV();
    }

    private Method getFindContoursHelperMethod(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, EMAMBluePrintCPP bluePrintCPP, MathExpressionProperties properties){
        Method method = new Method("findContoursHelper", "void");

        String typeName = getTypeOfFirstInput(mathMatrixNameExpressionSymbol, bluePrintCPP);
        String typeNameIn = "";

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
        String typeNameInConst = "const " + typeNameIn +"&";


        //add parameters
        Variable image = new Variable();
        method.addParameter(image, "image", "CommonMatrixType", typeNameInConst, MathConverter.curBackend.getIncludeHeaderName());
        Variable contours = new Variable();
        method.addParameter(contours, "contours", "CommonMatrixType", "std::vector<std::vector<cv::Point>>&", MathConverter.curBackend.getIncludeHeaderName());
        Variable mode = new Variable();
        method.addParameter(mode,"mode", "Integer", "int", "");
        Variable meth = new Variable();
        method.addParameter(meth, "method", "Integer","int", "");
        //add an instruction to the method
        method.addInstruction(methodBody(properties, typeNameIn));

        return method;
    }

    private Instruction methodBody(MathExpressionProperties properties, String typeNameIn) {
        return new Instruction() {
            @Override
            public String getTargetLanguageInstruction() {
                String finalInstruction ="";

                if(properties.isPreCV()){
                    finalInstruction += "    cv::findContours( image, contours, mode, method );\n";
                } else {
                    finalInstruction += "    cv::Mat srcCV;\n" +
                                        "    srcCV = to_cvmat<unsigned char>(image);\n" +
                                        "    cv::findContours( srcCV, contours, mode, method );\n";
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
