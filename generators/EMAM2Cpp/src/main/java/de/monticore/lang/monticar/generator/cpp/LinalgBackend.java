/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;
import de.monticore.lang.monticar.generator.MathBackend;
import de.se_rwth.commons.logging.Log;

/**
 */
public class LinalgBackend implements MathBackend {
    @Override
    public String getMatrixTypeName() {
        return null;
    }

    @Override
    public String getMatrixTypeUnsignedCharName() {
        return null;
    }

    @Override
    public String getCubeTypeName() {
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
        return null;
    }

    @Override
    public String getTubeAccessCommandName(){
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
