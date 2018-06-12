package de.monticore.lang.monticar.generator;

import de.monticore.lang.math.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;

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

    /**
     * Does the backend use 0-based or 1-based indexing for matrix element access?
     *
     * @return whether the backend uses zero based indexing (true) or one based indexing (false)
     */
    boolean usesZeroBasedIndexing();

}
