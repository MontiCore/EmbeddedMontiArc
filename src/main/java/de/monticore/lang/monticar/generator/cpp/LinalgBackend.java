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
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;
import de.monticore.lang.monticar.generator.MathBackend;
import de.se_rwth.commons.logging.Log;

/**
 * @author Sascha Schneiders
 */
public class LinalgBackend implements MathBackend {
    @Override
    public String getMatrixTypeName() {
        return null;
    }

    @Override
    public String getCubeTypeName() {
        return null;
    }

    @Override
    public String getMatrixInitString(int sizeN, int sizeM) {
        return "(" + sizeN + "," + sizeM + ");\n";
    }

    @Override
    public String getRowVectorTypeName() {
        return null;
    }

    @Override
    public String getColumnVectorTypeName() {
        return null;
    }

    @Override
    public String getColumnAccessCommandName() {
        return null;
    }

    @Override
    public String getRowAccessCommandName() {
        return null;
    }

    @Override
    public String getBackendName() {
        return null;
    }

    @Override
    public String getTransposeCommand() {
        return null;
    }

    @Override
    public String getIncludeHeaderName() {
        return null;
    }

    @Override
    public String getPowerOfString(MathArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        Log.debug("Not supported yet","Not Implemented");
        return null;
    }

    @Override
    public String getPowerOfString(MathArithmeticExpressionSymbol mathExpressionSymbol, String valueListString, String seperator) {
        Log.debug("Not supported yet","Not Implemented");
        return null;
    }

    @Override
    public String getPowerOfString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        Log.debug("Not supported yet","Not Implemented");
        return null;
    }

    @Override
    public String getPowerOfEEString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        Log.debug("Not supported yet","Not Implemented");
        return null;
    }

    @Override
    public String getDivisionEEString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        Log.debug("Not supported yet","Not Implemented");
        return null;
    }

    @Override
    public String getMultiplicationEEString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        Log.debug("Not supported yet","Not Implemented");
        return null;
    }

    @Override
    public boolean usesZeroBasedIndexing() {
        // TODO: check this! Do not know this backend...
        return false;
    }

    @Override
    public String getWholeNumberRowVectorTypeName() {
        Log.error("Not supported yet");
        return null;
    }

    @Override
    public String getWholeNumberColumnVectorTypeName() {
        Log.error("Not supported yet");
        return null;
    }

    @Override
    public String getWholeNumberMatrixTypeName() {
        Log.error("Not supported yet");
        return null;
    }

    @Override
    public String getWholeNumberCubeTypeName() {
        Log.error("Not supported yet");
        return null;
    }

    @Override
    public String getMathMatrixColonVectorString(MathMatrixVectorExpressionSymbol mathMatrixArithmeticExpressionSymbol) {
        Log.error("Not supported yet");
        return null;
    }
}
