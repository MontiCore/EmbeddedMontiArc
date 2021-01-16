/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;
import de.monticore.lang.monticar.generator.MathBackend;
import de.se_rwth.commons.logging.Log;

/**
 */
public class OctaveBackend implements MathBackend {
    public static final String NAME = "OctaveBackend";

    @Override
    public String getMatrixTypeName() {
        return "Matrix";
    }

    @Override
    public String getMatrixTypeUnsignedCharName() {
        Log.info("Matrix Unsigned Char Type not supported by currentBackend. ", getBackendName());
        return null;
    }

    @Override
    public String getCubeTypeName() {
        Log.info("Cube Type not supported by currentBackend. ", getBackendName());
        return null;
    }

    @Override
    public String getCubeUnsignedCharName() {
        Log.info("Cube Unsigned Char Type not supported by currentBackend. ", getBackendName());
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

    @Override public String getTubeAccessCommandName(){
        return "";
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
        Log.info("Octave does not support whole number matrices. Using real matrix instead!", "Octave");
        return getRowVectorTypeName();
    }

    @Override
    public String getWholeNumberColumnVectorTypeName() {
        Log.info("Octave does not support whole number matrices. Using real matrix instead!", "Octave");
        return getColumnVectorTypeName();
    }

    @Override
    public String getWholeNumberMatrixTypeName() {
        Log.info("Octave does not support whole number matrices. Using real matrix instead!", "Octave");
        return getMatrixTypeName();
    }

    @Override
    public String getWholeNumberCubeTypeName() {
        Log.info("Octave does not support whole number matrices. Using real matrix instead!", "Octave");
        return getCubeTypeName();
    }

    @Override
    public String getMathMatrixColonVectorString(MathMatrixVectorExpressionSymbol mathMatrixArithmeticExpressionSymbol) {
        Log.warn("Not supported.", mathMatrixArithmeticExpressionSymbol.getSourcePosition());
        return null;
    }
}
