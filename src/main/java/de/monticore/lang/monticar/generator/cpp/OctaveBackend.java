package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.math.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
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


}
