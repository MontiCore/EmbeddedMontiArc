/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;

/**
 */
public interface MathBackend {
    String getMatrixTypeName();

    String getMatrixTypeUnsignedCharName();

    String getCubeTypeName();

    String getCubeUnsignedCharName();

    String getMatrixInitString(int sizeN, int sizeM);

    String getRowVectorTypeName();

    String getColumnVectorTypeName();

    String getColumnAccessCommandName();

    String getRowAccessCommandName();

    String getTubeAccessCommandName();

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
