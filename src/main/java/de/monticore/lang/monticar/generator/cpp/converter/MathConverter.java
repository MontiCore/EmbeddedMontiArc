/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.monticar.generator.MathBackend;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.OctaveBackend;
import de.monticore.lang.monticar.generator.optimization.MathInformationRegister;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class MathConverter {

    public static MathBackend curBackend = new OctaveBackend();

    public static Variable getVariableFromBluePrint(MathMatrixNameExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        return getVariableFromBluePrint(mathExpressionSymbol.getNameToAccess(), bluePrintCPP);
    }

    public static Variable getVariableFromBluePrint(String namey, BluePrintCPP bluePrintCPP) {
        String name = EMAPortInstanceSymbol.getNameWithoutArrayBracketPart(namey);
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

    public static String getMatrixInitLine(Variable v, BluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getMatrixTypeName() + "(" + v.getDimensionalInformation().get(0) + "," + v.getDimensionalInformation().get(1) + ");\n";
    }

    public static String getRowVectorInitLine(Variable v, BluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getRowVectorTypeName() + "(" + v.getDimensionalInformation().get(1) + ");\n";
    }

    public static String getColumnVectorInitLine(Variable v, BluePrintCPP bluePrint) {
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

    public static String getCubeTypeInitLine(Variable v, BluePrintCPP bluePrint) {
        return String.format("%s = %s(%s, %s, %s);\n", MathInformationRegister.getVariableInitName(v, bluePrint), curBackend.getCubeTypeName(), v.getDimensionalInformation().get(0), v.getDimensionalInformation().get(1), v.getDimensionalInformation().get(2));
    }

    public static String getWholeNumberMatrixInitLine(Variable v, BluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getWholeNumberMatrixTypeName() + "(" + v.getDimensionalInformation().get(0) + "," + v.getDimensionalInformation().get(1) + ");\n";
    }

    public static String getWholeNumberRowVectorInitLine(Variable v, BluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getWholeNumberRowVectorTypeName() + "(" + v.getDimensionalInformation().get(1) + ");\n";
    }

    public static String getWholeNumberColumnVectorInitLine(Variable v, BluePrintCPP bluePrint) {
        return MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + curBackend.getWholeNumberColumnVectorTypeName() + "(" + v.getDimensionalInformation().get(0) + ");\n";
    }

    public static String getWholeNumberCubeInitLine(Variable v, BluePrintCPP bluePrint) {
        return String.format("%s = %s(%s, %s, %s);\n", MathInformationRegister.getVariableInitName(v, bluePrint), curBackend.getWholeNumberCubeTypeName(), v.getDimensionalInformation().get(0), v.getDimensionalInformation().get(1), v.getDimensionalInformation().get(2));
    }

    public static Optional<String> getInitLine(Variable v, BluePrintCPP bluePrint) {
        String initLine = null;
        if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getMatrixTypeName())) {
            initLine = MathConverter.getMatrixInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getRowVectorTypeName())) {
            initLine = MathConverter.getRowVectorInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getColumnVectorTypeName())) {
            initLine = MathConverter.getColumnVectorInitLine(v, bluePrint);
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getCubeTypeName())) {
            initLine = MathConverter.getCubeTypeInitLine(v, bluePrint);
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
}
