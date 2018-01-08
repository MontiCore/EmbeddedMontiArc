package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.math.math._symboltable.matrix.*;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.OctaveBackend;
import de.monticore.lang.monticar.generator.optimization.MathInformationRegister;

import de.monticore.lang.numberunit._ast.ASTUnitNumber;
import de.se_rwth.commons.logging.Log;

/**
 * @author Sascha Schneiders
 */
public class MathConverter {

    public static MathBackend curBackend = new OctaveBackend();

    public static Variable getVariableFromBluePrint(MathMatrixNameExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        return getVariableFromBluePrint(mathExpressionSymbol.getNameToAccess(), bluePrintCPP);
    }

    public static Variable getVariableFromBluePrint(String namey, BluePrintCPP bluePrintCPP) {
        String name = PortSymbol.getNameWithoutArrayBracketPart(namey);
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

    public static String getMatrixInitLine(Variable v, BluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getMatrixTypeName() + "(" + v.getDimensionalInformation().get(0) + "," + v.getDimensionalInformation().get(1) + ");\n";
    }

    public static String getRowVectorInitLine(Variable v, BluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getRowVectorTypeName() + "(" + v.getDimensionalInformation().get(1) + ");\n";
    }

    public static String getColumnVectorInitLine(Variable v, BluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getColumnVectorTypeName() + "(" + v.getDimensionalInformation().get(0) + ");\n";
    }

    public static String getConvertedUnitNumber(ASTUnitNumber unitNumber) {
        if (!unitNumber.getNumber().isPresent()) {
            Log.error("Number should be present");
        }
        if (unitNumber.getNumber().get().getDivisor().intValue() == 1) {
            return "" + unitNumber.getNumber().get().getDividend().intValue();
        } else {
            return "" + unitNumber.getNumber().get().doubleValue();
        }
    }
}
