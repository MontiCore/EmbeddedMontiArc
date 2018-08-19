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
package de.monticore.lang.monticar.generator;

import de.monticore.lang.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;

/**
 * @author Sascha Schneiders
 */
public interface MathBackend {
    String getMatrixTypeName();

    String getCubeTypeName();

    String getMatrixInitString(int sizeN, int sizeM);

    String getRowVectorTypeName();

    String getColumnVectorTypeName();

    String getColumnAccessCommandName();

    String getRowAccessCommandName();

    String getBackendName();

    String getTransposeCommand();

    String getIncludeHeaderName();

    String getPowerOfString(MathArithmeticExpressionSymbol mathExpressionSymbol, String valueListString);

    String getPowerOfString(MathArithmeticExpressionSymbol mathExpressionSymbol, String valueListString, String seperator);

    String getPowerOfString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString);

    String getPowerOfEEString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString);

    String getDivisionEEString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString);

    String getMultiplicationEEString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString);
    /**
     * Does the backend use 0-based or 1-based indexing for matrix element access?
     *
     * @return whether the backend uses zero based indexing (true) or one based indexing (false)
     */
    boolean usesZeroBasedIndexing();

    String getWholeNumberRowVectorTypeName();

    String getWholeNumberColumnVectorTypeName();

    String getWholeNumberMatrixTypeName();

    String getWholeNumberCubeTypeName();

    String getMathMatrixColonVectorString(MathMatrixVectorExpressionSymbol mathMatrixArithmeticExpressionSymbol);
}
