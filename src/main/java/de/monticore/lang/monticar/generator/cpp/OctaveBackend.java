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
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;
import de.monticore.lang.monticar.generator.MathBackend;
import de.se_rwth.commons.logging.Log;

/**
 * @author Sascha Schneiders
 */
public class OctaveBackend implements MathBackend {
    public static final String NAME = "OctaveBackend";

    @Override
    public String getMatrixTypeName() {
        return "Matrix";
    }

    @Override
    public String getCubeTypeName() {
        Log.info("Cube Type not supported by currentBackend. ", getBackendName());
        return null;
    }

    @Override
    public String getMatrixInitString(int sizeN, int sizeM) {
        return "(" + sizeN + "," + sizeM + ");\n";
    }

    @Override
    public String getRowVectorTypeName() {
        return "RowVector";
    }

    @Override
    public String getColumnVectorTypeName() {
        return "ColumnVector";
    }

    @Override
    public String getColumnAccessCommandName() {
        return "column";
    }

    @Override
    public String getRowAccessCommandName() {
        return "row";
    }

    @Override
    public String getBackendName() {
        return NAME;
    }

    @Override
    public String getTransposeCommand() {
        Log.error("Currently not supported in this backend");
        return null;
    }

    @Override
    public String getIncludeHeaderName() {
        return "octave/oct";
    }

    @Override
    public String getPowerOfString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        return OctaveHelper.getCallBuiltInFunctionFirstResult(mathExpressionSymbol.getLeftExpression(),
                "Fmpower", valueListString, false, 1);
    }

    @Override
    public String getPowerOfString(MathArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        return OctaveHelper.getCallBuiltInFunctionFirstResult(mathExpressionSymbol.getLeftExpression(),
                "Fmpower", valueListString, false, 1);
    }

    @Override
    public String getPowerOfString(MathArithmeticExpressionSymbol mathExpressionSymbol, String valueListString, String seperator) {
        return getPowerOfString(mathExpressionSymbol, valueListString);
    }

    @Override
    public String getPowerOfEEString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        return OctaveHelper.getCallOctaveFunctionFirstResult(mathExpressionSymbol.getLeftExpression(),
                "power", valueListString, false);
    }

    @Override
    public String getDivisionEEString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        return OctaveHelper.getCallOctaveFunctionFirstResult(mathExpressionSymbol.getLeftExpression(), "ldivide", valueListString, false);
    }

    @Override
    public String getMultiplicationEEString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        Log.warn("Backend deprecated");
        return OctaveHelper.getCallOctaveFunctionFirstResult(mathExpressionSymbol.getLeftExpression(), ".*", valueListString, false);
    }

    @Override
    public boolean usesZeroBasedIndexing() {
        return false;
    }

    @Override
    public String getWholeNumberRowVectorTypeName() {
        Log.warn("Octave does not support whole number matrices. Using real matrix instead!");
        return getRowVectorTypeName();
    }

    @Override
    public String getWholeNumberColumnVectorTypeName() {
        Log.warn("Octave does not support whole number matrices. Using real matrix instead!");
        return getColumnVectorTypeName();
    }

    @Override
    public String getWholeNumberMatrixTypeName() {
        Log.warn("Octave does not support whole number matrices. Using real matrix instead!");
        return getMatrixTypeName();
    }

    @Override
    public String getWholeNumberCubeTypeName() {
        Log.warn("Octave does not support whole number matrices. Using real matrix instead!");
        return getCubeTypeName();
    }

    @Override
    public String getMathMatrixColonVectorString(MathMatrixVectorExpressionSymbol mathMatrixArithmeticExpressionSymbol) {
        Log.warn("Not supported.", mathMatrixArithmeticExpressionSymbol.getSourcePosition());
        return null;
    }
}
