/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.MathExpressionProperties;
import de.monticore.lang.monticar.generator.cpp.OctaveBackend;
import de.monticore.lang.monticar.generator.optimization.MathInformationRegister;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 */
public class MathConverter {

    public static MathBackend curBackend = new OctaveBackend();

    public static Variable getVariableFromBluePrint(MathMatrixNameExpressionSymbol mathExpressionSymbol, EMAMBluePrintCPP bluePrintCPP) {
        return getVariableFromBluePrint(mathExpressionSymbol.getNameToAccess(), bluePrintCPP);
    }

    public static Variable getVariableFromBluePrint(String namey, EMAMBluePrintCPP bluePrintCPP) {

        String name = EMAPortSymbol.getNameWithoutArrayBracketPart(namey);
        Variable variable = bluePrintCPP.getVariable(name).orElse(null);
        return variable;
    }

    public static String getConstantConversion(MathExpressionSymbol mathExpressionSymbol) {
        if (mathExpressionSymbol.isMatrixExpression()) {
            MathMatrixExpressionSymbol matrixExpressionSymbol = (MathMatrixExpressionSymbol) mathExpressionSymbol;
            //TODO handle matrix/rowvector/columnvector conversion
            //mathExpressionSymbol.is
            if (matrixExpressionSymbol.isValueExpression()) {
                return getConstantConversion((MathMatrixArithmeticValueSymbol) matrixExpressionSymbol);
            }
            return "";
        } else {
            return mathExpressionSymbol.getTextualRepresentation();
        }

    }

    public static String getConstantConversion(MathMatrixArithmeticValueSymbol mathExpressionSymbol) {
        String constantName = "CONSTANTCONSTANTVECTOR" + getNextConstantConstantVectorID();

        String instructionString = getInstructionStringConstantVectorExpression(mathExpressionSymbol, constantName, TypeConverter.getTypeName(mathExpressionSymbol));

        TargetCodeInstruction instruction = new TargetCodeInstruction(instructionString);

        ComponentConverter.currentBluePrint.addInstructionToMethod(instruction, "init");

        Variable variable = new Variable();
        variable.setName(constantName);
        variable.setVariableType(TypeConverter.getVariableTypeForTargetLanguageTypeName(TypeConverter.getTypeName(mathExpressionSymbol)));

        ComponentConverter.currentBluePrint.addVariable(variable);

        return constantName;
    }

    public static String getInstructionStringConstantVectorExpression(MathMatrixArithmeticValueSymbol mathExpressionSymbol, String matrixName, String typeName) {
        String result = "";
        int column = 0;

        for (MathMatrixAccessOperatorSymbol symbol : mathExpressionSymbol.getVectors()) {
            Log.debug(symbol.getTextualRepresentation(), "Symbol:");
            int row = 0;
            for (MathMatrixAccessSymbol symbolAccess : symbol.getMathMatrixAccessSymbols()) {
                Log.debug("symbolAccess: " + symbolAccess.getTextualRepresentation(), "MathConverter");
                result += matrixName + "(" + column + "," + row + ") = ";
                if (symbolAccess.getMathExpressionSymbol().isPresent())
                    result += ExecuteMethodGenerator.generateExecuteCode(symbolAccess.getMathExpressionSymbol().get(), new ArrayList<>());
                else
                    result += symbolAccess.getTextualRepresentation();
                result += ";\n";
                ++row;
            }
            ++column;
        }
        String firstPart = matrixName + " = " + typeName;
        if (typeName.equals(curBackend.getRowVectorTypeName())) {
            firstPart += "(" + mathExpressionSymbol.getVectors().get(0).getMathMatrixAccessSymbols().size() + ");\n";
        } else if (typeName.equals(curBackend.getColumnVectorTypeName())) {
            firstPart += "(" + mathExpressionSymbol.getVectors().size() + ");\n";
        } else if (typeName.equals(curBackend.getMatrixTypeName())) {

            firstPart += curBackend.getMatrixInitString(mathExpressionSymbol.getVectors().size(),
                    mathExpressionSymbol.getVectors().get(0).getMathMatrixAccessSymbols().size());
        }
        return firstPart + result;
    }

    public static int CONSTANTCONSTANTVECTORID = 0;

    public static void resetIDs() {
        CONSTANTCONSTANTVECTORID = 0;
    }

    public static int getNextConstantConstantVectorID() {
        return CONSTANTCONSTANTVECTORID++;
    }

    public static String getMatrixInitLine(Variable v, EMAMBluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getMatrixTypeName() + "(" + v.getDimensionalInformation().get(0) + "," + v.getDimensionalInformation().get(1) + ");\n";
    }

    private static String getMatrixUnsignedCharInitLine(Variable v, EMAMBluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getMatrixTypeUnsignedCharName() + "(" + v.getDimensionalInformation().get(0) + "," + v.getDimensionalInformation().get(1) + ");\n";
    }

    public static String getRowVectorInitLine(Variable v, EMAMBluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getRowVectorTypeName() + "(" + v.getDimensionalInformation().get(1) + ");\n";
    }

    public static String getColumnVectorInitLine(Variable v, EMAMBluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getColumnVectorTypeName() + "(" + v.getDimensionalInformation().get(0) + ");\n";
    }

    public static String getConvertedUnitNumber(ASTNumberWithUnit unitNumber) {
        if (!unitNumber.getNumber().isPresent()) {
            Log.error("Number should be present");
        }
        if ((unitNumber.getNumber().get() % 1) == 0) {
            return "" + unitNumber.getNumber().get().intValue();
        } else {
            return "" + unitNumber.getNumber().get().doubleValue();
        }
    }

    public static String get4DCubeTypeInitLine(Variable v, EMAMBluePrintCPP bluePrint) {
        return String.format("%s = %s(%s, %s, %s);\n", MathInformationRegister.getVariableInitName(v, bluePrint), curBackend.getCubeTypeName(), v.getDimensionalInformation().get(1), v.getDimensionalInformation().get(2), v.getDimensionalInformation().get(3));
    }

    public static String getCubeTypeInitLine(Variable v, EMAMBluePrintCPP bluePrint) {
        return String.format("%s = %s(%s, %s, %s);\n", MathInformationRegister.getVariableInitName(v, bluePrint), curBackend.getCubeTypeName(), v.getDimensionalInformation().get(0), v.getDimensionalInformation().get(1), v.getDimensionalInformation().get(2));
    }

    private static String getCubeUnsignedCharInitLine(Variable v, EMAMBluePrintCPP bluePrint) {
        return String.format("%s = %s(%s, %s, %s);\n", MathInformationRegister.getVariableInitName(v, bluePrint), curBackend.getCubeUnsignedCharName(), v.getDimensionalInformation().get(0), v.getDimensionalInformation().get(1), v.getDimensionalInformation().get(2));
    }

    public static String getWholeNumberMatrixInitLine(Variable v, EMAMBluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getWholeNumberMatrixTypeName() + "(" + v.getDimensionalInformation().get(0) + "," + v.getDimensionalInformation().get(1) + ");\n";
    }

    public static String getWholeNumberRowVectorInitLine(Variable v, EMAMBluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getWholeNumberRowVectorTypeName() + "(" + v.getDimensionalInformation().get(1) + ");\n";
    }

    public static String getWholeNumberColumnVectorInitLine(Variable v, EMAMBluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getWholeNumberColumnVectorTypeName() + "(" + v.getDimensionalInformation().get(0) + ");\n";
    }

    public static String getWholeNumberCubeInitLine(Variable v, EMAMBluePrintCPP bluePrint) {
        return String.format("%s = %s(%s, %s, %s);\n", MathInformationRegister.getVariableInitName(v, bluePrint), curBackend.getWholeNumberCubeTypeName(), v.getDimensionalInformation().get(0), v.getDimensionalInformation().get(1), v.getDimensionalInformation().get(2));
    }

    public static Optional<String> getInitLine(Variable v, EMAMBluePrintCPP bluePrint) {
        String initLine = null;
        if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getMatrixTypeName())) {
            initLine = MathConverter.getMatrixInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getMatrixTypeUnsignedCharName())) {
            initLine = MathConverter.getMatrixUnsignedCharInitLine(v, bluePrint);
        }else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getRowVectorTypeName())) {
            initLine = MathConverter.getRowVectorInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getColumnVectorTypeName())) {
            initLine = MathConverter.getColumnVectorInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getCubeTypeName()) && v.getDimensionalInformation().size() == 4) {
            initLine = MathConverter.get4DCubeTypeInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getCubeTypeName())) {
            initLine = MathConverter.getCubeTypeInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getCubeUnsignedCharName())) {
            initLine = MathConverter.getCubeUnsignedCharInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getWholeNumberMatrixTypeName())) {
            initLine = MathConverter.getWholeNumberMatrixInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getWholeNumberRowVectorTypeName())) {
            initLine = MathConverter.getWholeNumberRowVectorInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getWholeNumberColumnVectorTypeName())) {
            initLine = MathConverter.getWholeNumberColumnVectorInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getWholeNumberCubeTypeName())) {
            initLine = MathConverter.getWholeNumberCubeInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals("double")) {
            initLine = "";
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals("bool")) {
            initLine = "";
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals("int")) {
            initLine = "";
        }

        return Optional.ofNullable(initLine);
    }

    public static void setPropertiesForMathExpression(List<MathExpressionSymbol> mathExpressionSymbols, MathExpressionSymbol mathExpressionSymbol, EMAMBluePrintCPP bluePrint, MathExpressionProperties properties){
        String nameOfFunction = ComponentConverter.getNameOfMathCommand(mathExpressionSymbol);
        MathCommand currentCommand = bluePrint.getMathCommandRegister().getMathCommand(nameOfFunction);
        int indexOfCurrentMathExpression = mathExpressionSymbols.indexOf(mathExpressionSymbol);
        if(currentCommand != null && currentCommand.isCVMathCommand()) {
            setPrePropertyForMathExpression(mathExpressionSymbols,mathExpressionSymbol, bluePrint, indexOfCurrentMathExpression, properties);
            setSucPropertyForMathExpression(mathExpressionSymbols, mathExpressionSymbol, bluePrint, indexOfCurrentMathExpression, properties);
        }
    }

    public static void setPrePropertyForMathExpression(List<MathExpressionSymbol> mathExpressionSymbols, MathExpressionSymbol currentMathExpressionSymbol, EMAMBluePrintCPP bluePrint, int currentMathExpressionIndex, MathExpressionProperties properties){
        for (int i = 0; i < currentMathExpressionIndex; i++) {
            MathExpressionSymbol preMathExpressionSymbol = mathExpressionSymbols.get(i);
            String nameOfMathCommand = ComponentConverter.getNameOfMathCommand(preMathExpressionSymbol);
            MathCommand mathCommand = bluePrint.getMathCommandRegister().getMathCommand(nameOfMathCommand);
            if(mathCommand != null && mathCommand.isCVMathCommand()){
                if(dependenceExists(preMathExpressionSymbol, currentMathExpressionSymbol)){
                    properties.setPreToCV();
                }
            }
        }
    }

    public static void setSucPropertyForMathExpression(List<MathExpressionSymbol> mathExpressionSymbols, MathExpressionSymbol currentMathExpressionSymbol, EMAMBluePrintCPP bluePrint, int currentMathExpressionIndex, MathExpressionProperties properties){
        int endIndex = mathExpressionSymbols.size();
        for (int i = currentMathExpressionIndex + 1; i < endIndex; i++) {
            MathExpressionSymbol sucMathExpressionSymbol = mathExpressionSymbols.get(i);
            String nameOfMathCommand = ComponentConverter.getNameOfMathCommand(sucMathExpressionSymbol);
            MathCommand mathCommand = bluePrint.getMathCommandRegister().getMathCommand(nameOfMathCommand);
            if(mathCommand != null && mathCommand.isCVMathCommand()){
                if(dependenceExists(currentMathExpressionSymbol, sucMathExpressionSymbol)){
                    properties.setSucToCV();
                }
            }
        }
    }

    public static boolean dependenceExists(MathExpressionSymbol preMathExpressionSymbol, MathExpressionSymbol sucMathExpressionSymbol){
        String outputOfPre = ComponentConverter.getNameOfOutput(preMathExpressionSymbol);
        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) ((MathAssignmentExpressionSymbol) sucMathExpressionSymbol).getExpressionSymbol();
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols()){
            String parameterName = accessSymbol.getTextualRepresentation();
            if(outputOfPre.equals(parameterName)){
                return true;
            }
        }
        return false;
    }
}
